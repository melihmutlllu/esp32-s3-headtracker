$ErrorActionPreference = "Stop"

$port = 4242
$udp = [System.Net.Sockets.UdpClient]::new()
$udp.Client.SetSocketOption(
  [System.Net.Sockets.SocketOptionLevel]::Socket,
  [System.Net.Sockets.SocketOptionName]::ReuseAddress,
  $true
)
$udp.Client.Bind([System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, $port))
$remote = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)

Write-Host "Listening UDP pose packets on 0.0.0.0:$port"
Write-Host "Expected packet: 6 little-endian doubles => tx, ty, tz, yaw, pitch, roll"
Write-Host "Close OpenTrack while using this receiver."

$packets = 0
$lastPrint = [DateTime]::MinValue

try {
  while ($true) {
    $data = $udp.Receive([ref]$remote)
    $packets++

    if ($data.Length -ne 48) {
      Write-Host ("{0}:{1} packet={2} bytes={3}" -f $remote.Address, $remote.Port, $packets, $data.Length)
      continue
    }

    $now = Get-Date
    if (($now - $lastPrint).TotalMilliseconds -lt 200) {
      continue
    }
    $lastPrint = $now

    $yaw = [BitConverter]::ToDouble($data, 24)
    $pitch = [BitConverter]::ToDouble($data, 32)
    $roll = [BitConverter]::ToDouble($data, 40)

    Write-Host ("{0}:{1} packets={2,6} yaw={3,8:N2} pitch={4,8:N2} roll={5,8:N2}" -f `
      $remote.Address, $remote.Port, $packets, $yaw, $pitch, $roll)
  }
}
finally {
  $udp.Close()
}
