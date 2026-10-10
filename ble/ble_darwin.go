package ble

import (
	"context"
	"errors"
	"fmt"
	"log"
	"os"
	"runtime"
	"strings"
	"sync"
	"time"
	"unsafe"

	"github.com/ebitengine/purego"
	"github.com/ebitengine/purego/objc"
)

// CoreBluetooth delivers every callback on one serial dispatch queue to a
// single delegate object. The delegate forwards them to the Go-side central
// (scan callbacks, adapter state) or to the Conn that owns the peripheral.

// CBManagerState values.
const (
	stateUnknown      = 0
	stateResetting    = 1
	stateUnsupported  = 2
	stateUnauthorized = 3
	statePoweredOff   = 4
	statePoweredOn    = 5
)

const writeWithResponse = 0 // CBCharacteristicWriteWithResponse

// writeTimeout bounds how long a write waits for the robot to acknowledge it.
const writeTimeout = 5 * time.Second

var (
	initOnce sync.Once
	initErr  error
	cm       central
)

type central struct {
	mgr objc.ID // CBCentralManager

	mu          sync.Mutex
	state       int
	stateChange chan struct{} // closed and replaced on every state change
	scanFn      func(Advertisement)
	peripherals map[string]objc.ID // retained, by identifier
	lastAdv     map[string]Advertisement
	conns       map[objc.ID]*Conn // by peripheral
}

var selCache sync.Map

func sel(name string) objc.SEL {
	if s, ok := selCache.Load(name); ok {
		return s.(objc.SEL)
	}
	s := objc.RegisterName(name)
	selCache.Store(name, s)
	return s
}

func class(name string) objc.ID { return objc.ID(objc.GetClass(name)) }

func send(id objc.ID, name string, args ...any) objc.ID { return id.Send(sel(name), args...) }

func nsString(s string) objc.ID {
	return send(class("NSString"), "stringWithUTF8String:", s)
}

func goString(ns objc.ID) string {
	if ns == 0 {
		return ""
	}
	return objc.Send[string](ns, sel("UTF8String"))
}

func goBytes(data objc.ID) []byte {
	if data == 0 {
		return nil
	}
	n := objc.Send[uint64](data, sel("length"))
	if n == 0 {
		return []byte{}
	}
	p := objc.Send[unsafe.Pointer](data, sel("bytes"))
	return append([]byte(nil), unsafe.Slice((*byte)(p), n)...)
}

func nsError(err objc.ID) error {
	if err == 0 {
		return nil
	}
	return errors.New("corebluetooth: " + goString(send(err, "localizedDescription")))
}

func cbUUID(s string) objc.ID {
	return send(class("CBUUID"), "UUIDWithString:", nsString(s))
}

func uuidString(obj objc.ID) string {
	return strings.ToLower(goString(send(send(obj, "UUID"), "UUIDString")))
}

func nsArray(objs ...objc.ID) objc.ID {
	arr := send(class("NSMutableArray"), "array")
	for _, o := range objs {
		send(arr, "addObject:", o)
	}
	return arr
}

func arrayItems(arr objc.ID) []objc.ID {
	if arr == 0 {
		return nil
	}
	n := objc.Send[uint64](arr, sel("count"))
	out := make([]objc.ID, n)
	for i := range out {
		out[i] = send(arr, "objectAtIndex:", uint64(i))
	}
	return out
}

// withPool runs fn inside an autorelease pool on a locked OS thread, so
// objects autoreleased by Go-side calls are freed.
func withPool(fn func()) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	pool := send(send(class("NSAutoreleasePool"), "alloc"), "init")
	defer send(pool, "drain")
	fn()
}

func setup() error {
	for _, path := range []string{
		"/System/Library/Frameworks/Foundation.framework/Foundation",
		"/System/Library/Frameworks/CoreBluetooth.framework/CoreBluetooth",
	} {
		if _, err := purego.Dlopen(path, purego.RTLD_NOW|purego.RTLD_GLOBAL); err != nil {
			return fmt.Errorf("ble: load %s: %w", path, err)
		}
	}
	libSystem, err := purego.Dlopen("/usr/lib/libSystem.B.dylib", purego.RTLD_NOW|purego.RTLD_GLOBAL)
	if err != nil {
		return fmt.Errorf("ble: load libSystem: %w", err)
	}
	var dispatchQueueCreate func(label string, attr uintptr) uintptr
	purego.RegisterLibFunc(&dispatchQueueCreate, libSystem, "dispatch_queue_create")

	type id = objc.ID
	type cmd = objc.SEL
	delegateClass, err := objc.RegisterClass("RootGoBLEDelegate", objc.GetClass("NSObject"), nil, nil, []objc.MethodDef{
		{Cmd: sel("centralManagerDidUpdateState:"), Fn: func(_ id, _ cmd, mgr id) {
			cm.setState(int(objc.Send[int64](mgr, sel("state"))))
		}},
		{Cmd: sel("centralManager:didDiscoverPeripheral:advertisementData:RSSI:"), Fn: func(_ id, _ cmd, _ id, p, adv, rssi id) {
			cm.discovered(p, adv, rssi)
		}},
		{Cmd: sel("centralManager:didConnectPeripheral:"), Fn: func(_ id, _ cmd, _ id, p id) {
			if c := cm.conn(p); c != nil {
				signal(c.connected, nil)
			}
		}},
		{Cmd: sel("centralManager:didFailToConnectPeripheral:error:"), Fn: func(_ id, _ cmd, _ id, p, err id) {
			if c := cm.conn(p); c != nil {
				signal(c.connected, cmp(nsError(err), errors.New("corebluetooth: failed to connect")))
			}
		}},
		{Cmd: sel("centralManager:didDisconnectPeripheral:error:"), Fn: func(_ id, _ cmd, _ id, p, err id) {
			if c := cm.conn(p); c != nil {
				c.shutdown(fmt.Errorf("%w: %v", ErrClosed, cmp(nsError(err), errors.New("disconnected"))))
			}
		}},
		{Cmd: sel("peripheral:didDiscoverServices:"), Fn: func(_ id, _ cmd, p, err id) {
			if c := cm.conn(p); c != nil {
				signal(c.step, nsError(err))
			}
		}},
		{Cmd: sel("peripheral:didDiscoverCharacteristicsForService:error:"), Fn: func(_ id, _ cmd, p, _ id, err id) {
			if c := cm.conn(p); c != nil {
				signal(c.step, nsError(err))
			}
		}},
		{Cmd: sel("peripheral:didUpdateNotificationStateForCharacteristic:error:"), Fn: func(_ id, _ cmd, p, _ id, err id) {
			if c := cm.conn(p); c != nil {
				signal(c.step, nsError(err))
			}
		}},
		{Cmd: sel("peripheral:didWriteValueForCharacteristic:error:"), Fn: func(_ id, _ cmd, p, _ id, err id) {
			if c := cm.conn(p); c != nil {
				signal(c.writeDone, nsError(err))
			}
		}},
		{Cmd: sel("peripheral:didUpdateValueForCharacteristic:error:"), Fn: func(_ id, _ cmd, p, ch, err id) {
			if c := cm.conn(p); c != nil && err == 0 {
				c.received(goBytes(send(ch, "value")))
			}
		}},
	})
	if err != nil {
		return fmt.Errorf("ble: register delegate: %w", err)
	}

	cm.stateChange = make(chan struct{})
	cm.peripherals = make(map[string]objc.ID)
	cm.lastAdv = make(map[string]Advertisement)
	cm.conns = make(map[objc.ID]*Conn)
	delegate := send(send(objc.ID(delegateClass), "alloc"), "init")
	queue := dispatchQueueCreate("rootgo.ble", 0) // serial
	withPool(func() {
		cm.mgr = send(send(class("CBCentralManager"), "alloc"), "initWithDelegate:queue:options:", delegate, queue, objc.ID(0))
	})
	if cm.mgr == 0 {
		return errors.New("ble: could not create CBCentralManager")
	}
	return nil
}

func cmp(err, fallback error) error {
	if err != nil {
		return err
	}
	return fallback
}

// signal does a non-blocking send on a 1-buffered channel.
func signal(ch chan error, err error) {
	select {
	case ch <- err:
	default:
	}
}

func (c *central) setState(s int) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.state = s
	close(c.stateChange)
	c.stateChange = make(chan struct{})
}

// ready initializes CoreBluetooth and waits for the adapter to power on.
func ready(ctx context.Context) error {
	initOnce.Do(func() { initErr = setup() })
	if initErr != nil {
		return initErr
	}
	for {
		cm.mu.Lock()
		s, changed := cm.state, cm.stateChange
		cm.mu.Unlock()
		switch s {
		case statePoweredOn:
			return nil
		case stateUnsupported:
			return errors.New("ble: this Mac does not support Bluetooth LE")
		case stateUnauthorized:
			return errors.New("ble: Bluetooth permission denied; allow your terminal app in System Settings > Privacy & Security > Bluetooth")
		case statePoweredOff:
			return errors.New("ble: Bluetooth is turned off")
		}
		select {
		case <-changed:
		case <-ctx.Done():
			return fmt.Errorf("ble: waiting for Bluetooth (state %d): %w", s, ctx.Err())
		}
	}
}

// debug, set by ROOTGO_BLE_DEBUG=1, logs every raw advertisement.
var debug = os.Getenv("ROOTGO_BLE_DEBUG") != ""

func (c *central) discovered(p, adv, rssi objc.ID) {
	if debug {
		log.Printf("ble: advertisement from %q: %s", goString(send(p, "name")), goString(send(adv, "description")))
	}
	a := Advertisement{
		ID:               strings.ToLower(goString(send(send(p, "identifier"), "UUIDString"))),
		Name:             goString(send(adv, "objectForKey:", nsString("kCBAdvDataLocalName"))),
		ManufacturerData: goBytes(send(adv, "objectForKey:", nsString("kCBAdvDataManufacturerData"))),
		RSSI:             int(objc.Send[int64](rssi, sel("integerValue"))),
	}
	a.Model = parseModel(a.ManufacturerData)
	if sd := send(adv, "objectForKey:", nsString("kCBAdvDataServiceData")); sd != 0 {
		a.State = parseRobotState(goBytes(send(sd, "objectForKey:", cbUUID(deviceInfoServiceUUID))))
	}

	c.mu.Lock()
	if _, ok := c.peripherals[a.ID]; !ok {
		c.peripherals[a.ID] = send(p, "retain")
	}
	// The robot splits its advertisement across two packets, so fill in what
	// this one lacks from the last one. The peripheral's own name property is
	// deliberately not used: macOS caches it and it goes stale after SetName.
	last := c.lastAdv[a.ID]
	if a.Name == "" {
		a.Name = last.Name
	}
	if a.ManufacturerData == nil {
		a.ManufacturerData, a.Model = last.ManufacturerData, last.Model
	}
	if a.State == nil {
		a.State = last.State
	}
	c.lastAdv[a.ID] = a
	fn := c.scanFn
	c.mu.Unlock()
	if fn != nil {
		fn(a)
	}
}

func (c *central) conn(p objc.ID) *Conn {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.conns[p]
}

// Scan reports Root robots to fn until ctx is done. A robot may be
// reported many times as its advertisement updates. fn runs on the
// Bluetooth callback queue and must not block. Only one scan may run at a
// time.
func Scan(ctx context.Context, fn func(Advertisement)) error {
	if err := ready(ctx); err != nil {
		return err
	}
	cm.mu.Lock()
	if cm.scanFn != nil {
		cm.mu.Unlock()
		return errors.New("ble: a scan is already running")
	}
	cm.scanFn = fn
	cm.mu.Unlock()

	withPool(func() {
		opts := send(class("NSDictionary"), "dictionaryWithObject:forKey:",
			send(class("NSNumber"), "numberWithBool:", true), nsString("kCBScanOptionAllowDuplicates"))
		send(cm.mgr, "scanForPeripheralsWithServices:options:", nsArray(cbUUID(RootServiceUUID)), opts)
	})
	<-ctx.Done()
	withPool(func() { send(cm.mgr, "stopScan") })

	cm.mu.Lock()
	cm.scanFn = nil
	cm.mu.Unlock()
	return nil
}

// Conn is a connection to one robot. It implements root.Transport.
type Conn struct {
	p      objc.ID
	rxChar objc.ID // we write here
	txChar objc.ID // the robot notifies here

	connected chan error
	step      chan error // services, characteristics and notify setup, in turn
	writeDone chan error
	rx        chan []byte

	wmu       sync.Mutex
	closeOnce sync.Once
	closed    chan struct{}
	err       error
}

// Connect connects to the robot with the given advertisement ID and
// subscribes to its packets.
func Connect(ctx context.Context, id string) (*Conn, error) {
	if err := ready(ctx); err != nil {
		return nil, err
	}
	id = strings.ToLower(id)
	cm.mu.Lock()
	p := cm.peripherals[id]
	cm.mu.Unlock()
	if p == 0 {
		withPool(func() {
			uuid := send(send(class("NSUUID"), "alloc"), "initWithUUIDString:", nsString(id))
			if uuid != 0 {
				if items := arrayItems(send(cm.mgr, "retrievePeripheralsWithIdentifiers:", nsArray(uuid))); len(items) > 0 {
					p = send(items[0], "retain")
				}
				send(uuid, "release")
			}
		})
		if p == 0 {
			return nil, fmt.Errorf("ble: unknown robot %s; scan for it first", id)
		}
		cm.mu.Lock()
		cm.peripherals[id] = p
		cm.mu.Unlock()
	}

	c := &Conn{
		p:         p,
		connected: make(chan error, 1),
		step:      make(chan error, 1),
		writeDone: make(chan error, 1),
		rx:        make(chan []byte, 256),
		closed:    make(chan struct{}),
	}
	cm.mu.Lock()
	if cm.conns[p] != nil {
		cm.mu.Unlock()
		return nil, fmt.Errorf("ble: already connected to %s", id)
	}
	cm.conns[p] = c
	cm.mu.Unlock()

	if err := c.setup(ctx); err != nil {
		c.Close()
		return nil, err
	}
	return c, nil
}

func (c *Conn) setup(ctx context.Context) error {
	wait := func(ch chan error, what string) error {
		select {
		case err := <-ch:
			if err != nil {
				return fmt.Errorf("ble: %s: %w", what, err)
			}
			return nil
		case <-c.closed:
			return c.err
		case <-ctx.Done():
			return fmt.Errorf("ble: %s: %w", what, ctx.Err())
		}
	}

	withPool(func() { send(cm.mgr, "connectPeripheral:options:", c.p, objc.ID(0)) })
	if err := wait(c.connected, "connect"); err != nil {
		return err
	}

	withPool(func() {
		send(c.p, "setDelegate:", send(cm.mgr, "delegate"))
		send(c.p, "discoverServices:", nsArray(cbUUID(UARTServiceUUID)))
	})
	if err := wait(c.step, "discover services"); err != nil {
		return err
	}

	var service objc.ID
	withPool(func() {
		for _, s := range arrayItems(send(c.p, "services")) {
			if uuidString(s) == UARTServiceUUID {
				service = s
				send(c.p, "discoverCharacteristics:forService:", nsArray(cbUUID(RXCharUUID), cbUUID(TXCharUUID)), s)
			}
		}
	})
	if service == 0 {
		return errors.New("ble: robot has no UART service")
	}
	if err := wait(c.step, "discover characteristics"); err != nil {
		return err
	}

	withPool(func() {
		for _, ch := range arrayItems(send(service, "characteristics")) {
			switch uuidString(ch) {
			case RXCharUUID:
				c.rxChar = ch
			case TXCharUUID:
				c.txChar = ch
			}
		}
		if c.txChar != 0 {
			send(c.p, "setNotifyValue:forCharacteristic:", true, c.txChar)
		}
	})
	if c.rxChar == 0 || c.txChar == 0 {
		return errors.New("ble: robot is missing the UART RX or TX characteristic")
	}
	return wait(c.step, "subscribe")
}

func (c *Conn) received(b []byte) {
	select {
	case c.rx <- b:
	case <-c.closed:
	}
}

func (c *Conn) shutdown(err error) {
	c.closeOnce.Do(func() {
		c.err = err
		close(c.closed)
		cm.mu.Lock()
		delete(cm.conns, c.p)
		cm.mu.Unlock()
	})
}

// ReadFrame returns the next notification from the robot.
func (c *Conn) ReadFrame() ([]byte, error) {
	select {
	case b := <-c.rx:
		return b, nil
	case <-c.closed:
		return nil, c.err
	}
}

// WriteFrame writes one frame and waits for the robot to acknowledge it.
func (c *Conn) WriteFrame(b []byte) error {
	c.wmu.Lock()
	defer c.wmu.Unlock()
	select {
	case <-c.closed:
		return c.err
	default:
	}
	// Discard a stale acknowledgement from a write that timed out.
	select {
	case <-c.writeDone:
	default:
	}
	withPool(func() {
		data := send(send(class("NSData"), "alloc"), "initWithBytes:length:", b, uint64(len(b)))
		send(c.p, "writeValue:forCharacteristic:type:", data, c.rxChar, writeWithResponse)
		send(data, "release")
	})
	select {
	case err := <-c.writeDone:
		return err
	case <-c.closed:
		return c.err
	case <-time.After(writeTimeout):
		return errors.New("ble: write not acknowledged")
	}
}

// Close disconnects from the robot.
func (c *Conn) Close() error {
	withPool(func() { send(cm.mgr, "cancelPeripheralConnection:", c.p) })
	c.shutdown(ErrClosed)
	return nil
}
