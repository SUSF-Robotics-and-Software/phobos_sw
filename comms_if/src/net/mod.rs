//! # Network Module
//!
//! This module provides networking abstractions over ZMQ, the networking library chosen for the 
//! software.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use std::{sync::{Arc, atomic::{AtomicBool, AtomicUsize}, atomic::Ordering}, thread};
use zmq::{Socket, Context, SocketType, SocketEvent};
// use log::debug;

// Export zmq
pub use zmq;

// ------------------------------------------------------------------------------------------------
// MACROS
// ------------------------------------------------------------------------------------------------

macro_rules! set_sockopts {
    ($socket:expr, $(($opt:ident, $val:expr)),+) => {
        $(
            $socket.$opt($val)
                .map_err(|e| MonitoredSocketError::SocketOptionError(stringify!($opt).into(), e))?;
        )+
    };
}

// ------------------------------------------------------------------------------------------------
// STATICS
// ------------------------------------------------------------------------------------------------

/// Number of monitors that are registered. Used to provide unique IDs for each mointor endpoint.
static NUM_MONITORS: AtomicUsize = AtomicUsize::new(0);

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// A zmq socket which is monitored providing additional information.
///
/// A background thread is run in order to monitor activity on the socket and update visible 
/// information to the user. Currently this is only whether or not the socket is actually connected.
pub struct MonitoredSocket {
    socket: Socket,

    join_handle: Option<thread::JoinHandle<()>>,

    _monitor_endpoint: String,

    shutdown: Arc<AtomicBool>,
    
    connected: Arc<AtomicBool>
}

/// Represents options which can be set on a monitored socket.
///
/// Most options here correspond to those found in the 
/// [`zmq_setsockopt`](http://api.zeromq.org/2-1:zmq-setsockopt) documentation.
pub struct SocketOptions {

    /// Indicates if the socket should bind itself to the endpoint. Servers should have this value
    /// set as `true`, clients should have it set as `false`.
    ///
    /// The default value is `false`.
    pub bind: bool,

    /// If true the `MonitoredSocket::new()` function will block until the socket is connected, or
    /// until the connect_timeout elapses. If the timeout elapses this function will return a
    /// `MonitoredSocketError::CouldNotConnect` error.
    ///
    /// The default value is `true`.
    pub block_on_first_connect: bool,

    /// `ZMQ_REQ_CORRELATE`: Match replies with requests
    pub req_correlate: bool,

    /// `ZMQ_REQ_RELAXED`: relax strict alternation between request and reply
    pub req_relaxed: bool,

    /// `ZMQ_LINGER`: Set linger period for socket shutdown
    pub linger: i32,

    /// `ZMQ_RECONNECT_IVL`: Set reconnection interval
    pub reconnect_ivl: i32,

    /// `ZMQ_RECONNECT_IVL_MAX`: Set maximum reconnection interval
    pub reconnect_ivl_max: i32,

    /// `ZMQ_CONNECT_TIMEOUT`: Set `connect()` timeout
    pub connect_timeout: i32,

    /// `ZMQ_RCVTIMEO`: Maximum time before a recv operation returns with `EAGAIN`
    pub recv_timeout: i32,

    /// `ZMQ_SNDTIMEO`: Maximum time before a send operation returns with `EAGAIN`
    pub send_timeout: i32,

    /// `ZMQ_HEARTBEAT_IVL`: Set interval between sending ZMTP heartbeats
    pub heartbeat_ivl: i32,

    /// `ZMQ_HEARTBEAT_TIMEOUT`: Set timeout for ZMTP heartbeats
    pub heartbeat_timeout: i32,

    /// `ZMQ_HEARTBEAT_TTL`: Set the TTL (time to live) value for ZMTP heartbeats
    pub heartbeat_ttl: i32
}

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

#[derive(thiserror::Error, Debug)]
pub enum MonitoredSocketError {
    #[error("Error creating the socket: {0}")]
    CreateSocketError(zmq::Error),

    #[error("Error enabling monitoring for the socket: {0}")]
    MonitoringEnableError(zmq::Error),

    #[error("Could not connect the socket: {0:?}")]
    CouldNotConnect(Option<zmq::Error>),

    #[error("Could not read event from monitor socket: {0}")]
    EventReadError(zmq::Error),

    #[error("Could not set the {0} socket option: {1}")]
    SocketOptionError(String, zmq::Error)
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl MonitoredSocket {
    /// Create a new monitored socket.
    ///
    /// ## Socket options
    ///
    /// The `socket_options` argument specifies the options that will be passed to the underlying
    /// zmq socket. For these options the defaults used by zmq are used.
    ///
    /// In addition some other options are available:
    /// - `bind`: If set the socket will bind itself to the endpoint rather than connect.
    ///    Servers should set this value to `true`. The default value is `false`.
    /// - `block_on_first_connect`: If set this function will block until a connection is 
    ///    established or the `connect_timeout` expires. Servers should set this value to `false`.
    ///    the default value is `true`.
    ///
    /// ## Arguments
    /// - `ctx`: the zmq context which will be used to create the socket
    /// - `socket_type`: the type of zmq socket to create
    /// - `socket_options`: a [`SocketOptions`] struct specifying how to configure the socket
    /// - `endpoint`: a zmq endpoint string, such as `"tcp://localhost:4000"`
    pub fn new(
        ctx: &Context, 
        socket_type: SocketType,
        socket_options: SocketOptions,
        endpoint: &str
    ) -> Result<Self, MonitoredSocketError> {
        // Create atomics
        let shutdown = Arc::new(AtomicBool::new(false));
        let connected = Arc::new(AtomicBool::new(false));

        // Create socket
        let socket = ctx.socket(socket_type)
            .map_err(|e| MonitoredSocketError::CreateSocketError(e))?;

        // Create monitor endpoint
        let monitor_endpoint = format!(
            "inproc://monitor_{}", 
            NUM_MONITORS.fetch_add(1, Ordering::Relaxed)
        );

        // Enable, create, and connect monitor
        socket.monitor(&monitor_endpoint, SocketEvent::ALL as i32)
            .map_err(|e| MonitoredSocketError::MonitoringEnableError(e))?;
        let monitor = ctx.socket(zmq::PAIR)
            .map_err(|e| MonitoredSocketError::CreateSocketError(e))?;
        monitor.connect(&monitor_endpoint)
            .map_err(|e| MonitoredSocketError::CouldNotConnect(Some(e)))?;

        // Set the options on the socket
        socket_options.set(&socket)?;

        // Connect or bind the socket to it's endpoint
        match socket_options.bind {
            false => socket.connect(endpoint),
            true=> socket.bind(endpoint)
        }.map_err(|e| MonitoredSocketError::CouldNotConnect(Some(e)))?;

        // If the block on first connect flag is set, and this is a client, wait for the monitor to
        // signal connection
        if socket_options.block_on_first_connect
        {
            loop {
                let event = read_event(&monitor)
                    .map_err(|e| MonitoredSocketError::EventReadError(e))?;

                match event {
                    SocketEvent::CONNECTED => break,
                    SocketEvent::CONNECT_DELAYED => continue,
                    _ => return Err(MonitoredSocketError::CouldNotConnect(None))
                }
            }

            // Set the connected bool to true here since it must have happend
            connected.store(true, Ordering::Relaxed);
        }

        // Create clones for use by the monitor thread
        let shutdown_clone = shutdown.clone();
        let connected_clone = connected.clone();
        let monitor_endpoint_clone = monitor_endpoint.clone();

        // Spawn the monitor thread
        let join_handle = thread::spawn(move || monitor_socket(
            monitor, 
            monitor_endpoint_clone,
            shutdown_clone, 
            connected_clone
        ));

        // Create self
        Ok(Self {
            socket,
            join_handle: Some(join_handle),
            _monitor_endpoint: monitor_endpoint,
            shutdown,
            connected
        })
    }

    /// Return if the socket is connected or not.
    pub fn connected(&self) -> bool {
        self.connected.load(Ordering::Relaxed)
    }
}

impl Drop for MonitoredSocket {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::Relaxed);
        
        // debug!("MonitoredSocket shutdown sent");
        
        if let Some(_jh) =  self.join_handle.take() {
            // TODO: Currently this hangs, probably because the monitor thread is waiting for an
            // event but none is coming, add timeout?
            // debug!("Waiting for join");
            // jh.join().ok();
        }
    }
}

impl std::ops::Deref for MonitoredSocket {
    type Target = Socket;

    fn deref(&self) -> &Self::Target {
        &self.socket
    }
}

impl std::ops::DerefMut for MonitoredSocket {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.socket
    }
}

impl SocketOptions {
    /// Set these options on the given socket.
    pub fn set(&self, socket: &Socket) -> Result<(), MonitoredSocketError> {

        // Set all the socket options, we use a macro here to make the error handling nice and
        // easy
        set_sockopts!(
            socket,
            (set_connect_timeout, self.connect_timeout),
            (set_heartbeat_ivl, self.heartbeat_ivl),
            (set_heartbeat_timeout, self.heartbeat_timeout),
            (set_heartbeat_ttl, self.heartbeat_ttl),
            (set_linger, self.linger),
            (set_reconnect_ivl, self.reconnect_ivl),
            (set_reconnect_ivl_max, self.reconnect_ivl_max),
            (set_rcvtimeo, self.recv_timeout),
            (set_sndtimeo, self.send_timeout)
        );

        // If the socket is a req type set the req-specific options
        if let Ok(SocketType::REQ) = socket.get_socket_type() {
            set_sockopts!(
                socket,
                (set_req_correlate, self.req_correlate),
                (set_req_relaxed, self.req_relaxed)
            );
        }

        Ok(())
    }
}

impl Default for SocketOptions {
    fn default() -> Self {
        // Defaults for sockopts taken from http://api.zeromq.org/4-2:zmq-setsockopt
        Self {
            bind: false,
            block_on_first_connect: true,
            connect_timeout: 0,
            heartbeat_ivl: 0,
            heartbeat_timeout: 0,
            heartbeat_ttl: 0,
            linger: 30_000,
            reconnect_ivl: 100,
            reconnect_ivl_max: 0,
            recv_timeout: -1,
            req_correlate: false,
            req_relaxed: false,
            send_timeout: 0
        }
    }
}

// ------------------------------------------------------------------------------------------------
// PRIVATE FUNCTIONS
// ------------------------------------------------------------------------------------------------

/// Read an event from a socket.
fn read_event(socket: &Socket) -> Result<SocketEvent, zmq::Error> {
    
    let msg = socket.recv_msg(0)?;
        
    // TODO: could be simplified by using `TryInto` (since 1.34)
    let event = u16::from_ne_bytes([msg[0], msg[1]]);

    assert!(
        socket.get_rcvmore()?,
        "Monitor socket should have two messages per event"
    );

    // the address, we'll ignore it
    let _ = socket.recv_msg(0)?;

    Ok(SocketEvent::from_raw(event))
}

fn monitor_socket(
    monitor: Socket,
    monitor_endpoint: String,
    shutdown: Arc<AtomicBool>,
    connected: Arc<AtomicBool>
) {
    // So long as the shutdown isn't requested
    while !shutdown.load(Ordering::Relaxed) {
        // Read the next event from the monitor
        let event = read_event(&monitor)
            .expect(&format!(
                "Error reading event from monitor {}",
                monitor_endpoint
            ));

        // Raise any flags required by the event
        match event {
            SocketEvent::CONNECTED => connected.store(true, Ordering::Relaxed),
            SocketEvent::DISCONNECTED => connected.store(false, Ordering::Relaxed),
            _ => ()
        }
    }
}
