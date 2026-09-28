#pragma once

static const char setupPage[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LitterSense Wi-Fi Setup</title>
<style>body{font:16px system-ui;background:#101412;color:#edf8f2;margin:0}main{max-width:440px;margin:auto;padding:28px 20px}h1{font-size:26px}p{line-height:1.5;color:#bbc9c1}label{display:block;margin-top:20px}input,button{box-sizing:border-box;width:100%;padding:14px;border-radius:10px;font:inherit;margin-top:8px}input{background:#202923;color:white;border:1px solid #627369}button{background:#2ad39a;color:#08271b;border:0;font-weight:700}button:disabled{opacity:.6}#status{padding:12px 0;min-height:48px}a{color:#50e5b0}</style>
<main><h1>LitterSense Wi-Fi Setup</h1><p>Choose the Wi-Fi at this location. This page works without internet.</p>
<form id="setup" method="post" action="/provision">
<input type="hidden" name="csrf" value="{{CSRF}}">
<label for="ssid">Wi-Fi name (SSID)</label><input id="ssid" name="ssid" maxlength="32" required autocapitalize="none" autocomplete="off" spellcheck="false">
<label for="password">Wi-Fi password</label><input id="password" name="password" type="password" maxlength="64" autocomplete="current-password">
<p>Use a 2.4 GHz personal Wi-Fi network. Leave the password blank for an open network. Networks requiring a browser login or enterprise account are not supported.</p>
<button id="connect">Connect litterbox</button></form>
<p id="status" role="status" aria-live="polite">Credentials are saved on the litterbox only after a successful connection.</p>
<p>After connecting, rejoin your usual Wi-Fi. The litterbox remembers this network after a power cycle. At a new place, repeat this setup.</p></main>
<script>
const form=document.getElementById('setup'), status=document.getElementById('status'), button=document.getElementById('connect');
const messages={idle:'Enter the Wi-Fi details for this location.',connecting:'Testing Wi-Fi. Stay connected to LitterSense-Setup; this can take 30 seconds.',connected:'Connected and saved! Rejoin your usual Wi-Fi. The setup hotspot will close shortly.',failed:'Could not connect. Check the name, password and 2.4 GHz signal, then try again. Previous saved details were kept.',storage_error:'Wi-Fi connected, but saving failed. Please retry; these details will not survive a restart.'};
async function refresh(){try{const r=await fetch('/setup/status',{cache:'no-store'});if(!r.ok)return;const s=await r.json();status.textContent=messages[s.state]||'Setup available.';button.disabled=s.state==='connecting'||s.state==='connected';}catch{status.textContent='Connection interrupted. If setup has finished, rejoin your usual Wi-Fi. Otherwise reconnect to LitterSense-Setup and reload this page.';}}
form.addEventListener('submit',async e=>{e.preventDefault();button.disabled=true;try{const r=await fetch('/provision',{method:'POST',body:new URLSearchParams(new FormData(form))});if(!r.ok)throw new Error(await r.text());document.getElementById('password').value='';await refresh();}catch(e){status.textContent=e.message||'Reconnect to the setup Wi-Fi and retry.';button.disabled=false;}});
setInterval(refresh,2000);
</script></html>)HTML";
