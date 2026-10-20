<#
.SYNOPSIS
    Forwards Motive's NatNet stream from Windows into WSL over TCP.

.DESCRIPTION
    Under WSL2 mirrored networking, Windows->WSL inbound UDP (Motive's NatNet
    stream) is not delivered into the VM, but a WSL process can connect OUT to
    a Windows server over localhost. This relay bridges that gap on the Windows
    side and re-serves each datagram, length-prefixed, over a TCP socket that
    the WSL `mocap_zenoh_bridge` connects to. The bridge currently uses
    127.0.0.1:1512, so keep TcpPort at its default.

    Serves ANY NUMBER of concurrent WSL clients: each datagram is fanned out
    to every attached client, and a client that disconnects is dropped without
    disturbing the others. This matters because the launcher
    (`cargo run --locked`, with a hardware or hybrid agent) spawns its own
    `mocap_zenoh_bridge`, so a
    standalone bridge (e.g. the drift-logging workflow) may legitimately be
    attached at the same time -- with a single-client relay the second bridge
    would sit in the TCP backlog forever, "connected" but receiving nothing.

    Two modes:
      unicast   (default) -- sends a NAT_CONNECT handshake to Motive's command
                port so Motive unicasts the stream back to this relay, plus a
                periodic keepalive so Motive doesn't drop the client as idle.
                The subscription is kept alive from startup, whether or not a
                WSL client is currently attached (the relay is a dumb pipe;
                datagrams with no consumer are discarded).
                Works entirely over loopback when Motive's Local Interface is
                set to loopback; the most reliable same-machine setup.
      multicast -- passively joins Motive's multicast group (no handshake). The
                join interface must match Motive's Local Interface.

    The relay understands nothing about NatNet beyond the 4-byte command
    framing; all parsing/publishing/recording stays in the WSL Rust application.

    Wire framing to WSL (per datagram): 4-byte little-endian length, then that
    many raw NatNet bytes -- exactly one UDP datagram as received from Motive.

.EXAMPLE
    # Unicast + loopback (default). In Motive: Transmission Type=Unicast,
    # Local Interface=loopback, Rigid Bodies on.
    powershell -ExecutionPolicy Bypass -File .\natnet_relay.ps1

.EXAMPLE
    # Multicast fallback. In Motive: Transmission Type=Multicast,
    # Local Interface=198.51.100.191.
    powershell -ExecutionPolicy Bypass -File .\natnet_relay.ps1 -Mode multicast -McastIface 198.51.100.191
#>
param(
    [ValidateSet("unicast", "multicast")]
    [string]$Mode       = "unicast",
    [string]$CmdHost    = "127.0.0.1",
    [int]$CmdPort       = 1510,
    [int]$DataPort      = 1511,
    [int]$TcpPort       = 1512,
    [string]$McastGroup = "239.255.42.99",
    [string]$McastIface = "127.0.0.1"
)

# NatNet command message IDs.
$NAT_CONNECT = 0
$NAT_REQUEST = 2
$EMPTY = [byte[]]@()

# Builds a framed NatNet command: 2-byte ID + 2-byte length (both LE) + payload.
function New-NatPacket([int]$MessageId, [byte[]]$Payload) {
    $bytes = New-Object System.Collections.Generic.List[byte]
    $bytes.AddRange([BitConverter]::GetBytes([uint16]$MessageId))
    $bytes.AddRange([BitConverter]::GetBytes([uint16]$Payload.Length))
    if ($Payload.Length -gt 0) { $bytes.AddRange($Payload) }
    return $bytes.ToArray()
}

# --- Data socket: bind the NatNet data port (receives the Frame-of-Data stream). ---
$udp = New-Object System.Net.Sockets.UdpClient
$udp.Client.SetSocketOption(
    [System.Net.Sockets.SocketOptionLevel]::Socket,
    [System.Net.Sockets.SocketOptionName]::ReuseAddress, $true)
$udp.Client.Bind((New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, $DataPort)))
$udp.Client.ReceiveTimeout = 1000   # ms; lets the loop tick to accept clients / send keepalives

# --- Mode setup. ---
$cmd = $null
$subPayload = $EMPTY
if ($Mode -eq "unicast") {
    $cmd = New-Object System.Net.Sockets.UdpClient
    $cmd.Connect($CmdHost, $CmdPort)
    $subPayload = [System.Text.Encoding]::ASCII.GetBytes("SubscribeToData,RigidBody,all") + [byte]0
} else {
    $udp.JoinMulticastGroup(
        [System.Net.IPAddress]::Parse($McastGroup),
        [System.Net.IPAddress]::Parse($McastIface))
}

# --- TCP server: WSL bridges connect here (any number, concurrently). ---
$listener = New-Object System.Net.Sockets.TcpListener([System.Net.IPAddress]::Any, $TcpPort)
$listener.Start()
if ($Mode -eq "unicast") {
    Write-Host "Relay up: unicast from Motive ${CmdHost}:${CmdPort} -> TCP $TcpPort."
} else {
    Write-Host "Relay up: multicast $McastGroup on $McastIface (UDP $DataPort) -> TCP $TcpPort."
}
Write-Host "In WSL run the stack with WSL Relay selected: cargo run --locked"
Write-Host "Waiting for WSL client(s) (Ctrl-C to stop)..."

# Attached clients: @{ Client; Stream; Label } records, fanned out per datagram.
$clients = New-Object System.Collections.ArrayList
$remote = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, 0)
$count = 0
$lastKeepalive = [DateTime]::MinValue

while ($true) {
    # Accept newly connected WSL clients without blocking the forward loop.
    while ($listener.Pending()) {
        $client = $listener.AcceptTcpClient()
        $client.NoDelay = $true
        $entry = [pscustomobject]@{
            Client = $client
            Stream = $client.GetStream()
            Label  = "$($client.Client.RemoteEndPoint)"
        }
        [void]$clients.Add($entry)
        Write-Host "WSL client connected from $($entry.Label) ($($clients.Count) attached)."
    }

    # Keep Motive's unicast subscription alive from startup (the "streams for
    # ~2 seconds then stops" unicast idle timeout); re-request the data
    # subscription on the first tick.
    if ($Mode -eq "unicast" -and ((Get-Date) - $lastKeepalive).TotalSeconds -ge 1) {
        if ($lastKeepalive -eq [DateTime]::MinValue) {
            $request = New-NatPacket $NAT_REQUEST $subPayload
            $cmd.Send($request, $request.Length) | Out-Null
        }
        $connect = New-NatPacket $NAT_CONNECT $EMPTY
        $cmd.Send($connect, $connect.Length) | Out-Null
        $lastKeepalive = Get-Date
    }

    $data = $null
    try {
        $data = $udp.Receive([ref]$remote)   # one NatNet datagram
    } catch [System.Net.Sockets.SocketException] {
        # A receive timeout is the expected idle tick; anything else is real.
        if ($_.Exception.SocketErrorCode -ne [System.Net.Sockets.SocketError]::TimedOut) { throw }
    }
    if (-not $data) { continue }

    $count++
    if ($count -le 5 -or ($count % 240) -eq 0) {
        Write-Host "forwarded $count datagrams to $($clients.Count) client(s) (last $($data.Length) bytes)"
    }
    if ($clients.Count -eq 0) { continue }

    $len = [BitConverter]::GetBytes([int]$data.Length)
    $dead = @()
    foreach ($entry in $clients) {
        try {
            $entry.Stream.Write($len, 0, 4)
            $entry.Stream.Write($data, 0, $data.Length)
        } catch {
            $dead += $entry
        }
    }
    foreach ($entry in $dead) {
        Write-Host "WSL client $($entry.Label) disconnected ($($clients.Count - 1) attached)."
        $clients.Remove($entry)
        try { $entry.Client.Close() } catch {}
    }
}
