# Run on a Windows PC connected ONLY to LitterSense (disconnect Ethernet/VPN/other Wi-Fi).
$ErrorActionPreference = 'Stop'
$ap = Get-NetIPConfiguration | Where-Object { $_.IPv4DefaultGateway.NextHop -eq '192.168.4.1' }
if (-not $ap) { throw 'Connect this PC to LitterSense using DHCP first.' }
$dns = $ap.DNSServer.ServerAddresses
if ('1.1.1.1' -notin $dns) { throw 'DHCP did not advertise the configured DNS server (1.1.1.1).' }
$answer = Resolve-DnsName example.com -Server 1.1.1.1 -Type A -DnsOnly
if (-not ($answer | Where-Object IPAddress)) { throw 'DNS lookup failed.' }
$response = Invoke-WebRequest https://example.com -UseBasicParsing -TimeoutSec 20
if ($response.StatusCode -ne 200) { throw 'Public HTTPS request failed.' }
Write-Output 'PASS: DHCP DNS, public DNS lookup and public HTTPS. Run the phone and stream tests too.'
