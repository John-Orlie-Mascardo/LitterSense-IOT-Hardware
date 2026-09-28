#include <cassert>
#include <iostream>
#include "../littersense_rfid_test.ino"
int main(){
 assert(rfidTransportAllowed("http://192.168.68.108:3000/api/sensors", "", true));
 assert(!rfidTransportAllowed("http://192.168.68.108:3000/api/sensors", "", false));
 assert(!rfidTransportAllowed("https://example.com/api/sensors", "", true));
 assert(rfidTransportAllowed("https://example.com/api/sensors", "certificate", false));
 assert(!rfidTransportAllowed("ftp://example.com", "certificate", true));
 uint8_t epcPayload[]={0,0x08,0,0x01,0xAB,0,0};
 assert(extractEPC(epcPayload,7)=="01AB");
 assert(extractEPC(epcPayload,6)=="");
 epcPayload[1]=0x10; assert(extractEPC(epcPayload,7)=="");
 setup();
 handleTag("A",1000); assert(!inside && pendingTag=="A" && !scanArmed);
 handleTag("B",1100); assert(pendingTag=="A");
 testConfirmedGeneration=pendingRequest;
 handlePendingEntry(1200); assert(inside && entryTime==1200 && pendingTag=="");
 handleTag("A",2000); assert(inside && entryTime==1200);
 handleNoTag(2100); handleNoTag(5099); assert(!scanArmed);
 handleNoTag(5100); assert(scanArmed && inside);
 handleTag("B",5200); assert(inside && activeTag=="A" && scanArmed);
 handleTag("A",6000); assert(!inside && !scanArmed);
 handleNoTag(6100); resetClearWindow(); handleNoTag(9200); assert(!scanArmed);
 handleNoTag(12200); assert(scanArmed);
 handleTag("A",UINT32_MAX-1000);
 testConfirmedGeneration=pendingRequest;
 handlePendingEntry(UINT32_MAX-800);
 handleNoTag(UINT32_MAX-500); handleNoTag(2499); assert(scanArmed);
 assert(uint32_t(3000-entryTime)==3801);
 handleTag("A",3000); assert(!inside);
 // No confirmation: no entry, no exit event, and no automatic retry while held.
 handleNoTag(3100); handleNoTag(6100);
 handleTag("A",6200); const uint32_t expiredRequest=pendingRequest;
 handlePendingEntry(11199); assert(!inside && pendingTag=="A");
 testConfirmedGeneration=pendingRequest;
 handlePendingEntry(11200); assert(!inside && pendingTag=="" && !scanArmed);
 handleTag("A",11300); assert(pendingTag=="");
 handleNoTag(11400); handleNoTag(14400);
 handleTag("B",14500);
 testConfirmedGeneration=expiredRequest;
 handlePendingEntry(14600); assert(!inside && pendingTag=="B");
 handlePendingEntry(19500); assert(!inside && pendingTag=="");
 // Timeout across rollover, even with a response arriving at the deadline.
 scanArmed=true;
 handleTag("A",UINT32_MAX-1000);
 handlePendingEntry(3998); assert(pendingTag=="A");
 testConfirmedGeneration=pendingRequest;
 handlePendingEntry(3999); assert(!inside && pendingTag=="");
 uint8_t t=0,c=0,p[128];
 rfid.rx={0xBB,0x01,0xFF};
 assert(readFrame(t,c,p,128)==-2 && used==3);
 rfid.rx={0x00,0x01,0x15,0x16,0x7E};
 assert(readFrame(t,c,p,128)==1 && c==0xFF && p[0]==0x15 && used==0);
 rfid.rx={0xBB,0x01,0xFF,0,1,0x15,0,0x7E};
 assert(readFrame(t,c,p,128)==-1);
 assert(used==0 && expected==0);
 used=expected=0; clearWindowStarted=true;
 awaitingReply=true; powerPending=false; commandSentAt=0; clockMs=300;
 loop(); assert(!clearWindowStarted);
 std::cout<<"PASS: duplicate scans, rearm boundary, different tag, interrupted removal, rollover, split UART frame, checksum, timeout\n";
}
