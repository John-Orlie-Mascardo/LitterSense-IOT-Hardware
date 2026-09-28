# Run on a Windows PC connected ONLY to LitterSense (disconnect Ethernet/VPN/other Wi-Fi).
$ErrorActionPreference = 'Stop'
$ap = Get-NetIPConfiguration | Where-Object { $_.IPv4DefaultGateway.NextHop -in @('192.168.4.1', '192.168.50.1', '10.42.0.1', '172.31.250.1') }
if (-not $ap) { throw 'Connect this PC to LitterSense using DHCP first.' }
$dns = @($ap.DNSServer.ServerAddresses | Where-Object { $_ -match '^\d+\.\d+\.\d+\.\d+$' })
if (-not $dns.Count) { throw 'DHCP did not advertise an IPv4 DNS server.' }
$answer = Resolve-DnsName example.com -Server $dns[0] -Type A -DnsOnly
if (-not ($answer | Where-Object IPAddress)) { throw 'DNS lookup failed.' }
$response = Invoke-WebRequest https://example.com -UseBasicParsing -TimeoutSec 20
if ($response.StatusCode -ne 200) { throw 'Public HTTPS request failed.' }
Write-Output 'PASS: DHCP DNS, public DNS lookup and public HTTPS. Run the phone and stream tests too.'
