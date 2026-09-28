# Run with this PC connected to LitterSense-Setup. Does not change credentials.
$ErrorActionPreference = 'Stop'
$base = 'http://192.168.4.1'
$page = Invoke-WebRequest "$base/" -UseBasicParsing -TimeoutSec 10
if ($page.StatusCode -ne 200 -or $page.Content -notmatch 'LitterSense Wi-Fi Setup' -or $page.Content -notmatch 'name="csrf"') {
    throw 'Setup root did not serve the Wi-Fi form.'
}
$status = Invoke-RestMethod "$base/setup/status" -TimeoutSec 10
if ($status.state -notin @('idle', 'connecting', 'connected', 'failed', 'storage_error')) {
    throw 'Setup status was inaccessible or invalid.'
}
$probes = @(
    @('connectivitycheck.gstatic.com', '/generate_204'),
    @('captive.apple.com', '/hotspot-detect.html'),
    @('www.msftconnecttest.com', '/connecttest.txt'),
    @('captive.apple.com', '/')
)
foreach ($probe in $probes) {
    $answer = Resolve-DnsName $probe[0] -Server 192.168.4.1 -Type A -DnsOnly
    if ('192.168.4.1' -notin $answer.IPAddress) { throw "Captive DNS failed for $($probe[0])." }
    $reply = & curl.exe --silent --show-error --noproxy '*' --max-time 10 --output NUL --write-out '%{http_code} %{redirect_url}' --header "Host: $($probe[0])" "$base$($probe[1])"
    if ($LASTEXITCODE -ne 0 -or $reply -ne '302 http://192.168.4.1/') { throw "Captive redirect failed for $($probe[0]): $reply" }
}
Write-Output 'PASS: setup form, status, captive DNS, and Android/Apple/Windows probe redirects. Verify the actual phone notification separately.'
