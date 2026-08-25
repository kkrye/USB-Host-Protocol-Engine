`include "USBPkg.pkg"

// Wrapper for USB bus states. Notice that enum Z can only be driven, not read
typedef enum logic [1:0]
  {BS_J = 2'b10, BS_K = 2'b01, BS_SE0 = 2'b00, BS_SE1 = 2'b11, BS_NC = 2'bzz}
  bus_state_t;

module USBHost (
  USBWires wires,
  input logic clock, reset_n
);

  //control signals also used in prelab
  pkt_t pkt;
  logic pkt_valid;
  logic done;
  logic CRC5_bit_out;
  logic [4:0] CRC5_val;
  logic send0;
  logic BS_bit_out;
  usb_bus_t usb_bus;
  logic wires_DP;
  logic wires_DM;
  logic wires_stateP;
  logic wires_stateM;
  logic sending;
  logic [4:0] CRCstate;
  logic sendPID;
  logic sendADDR;
  logic sendENDP;
  logic sendCRC;
  logic sendPAYLOAD;
  logic idle;

//commented out prelab request to avoid multiple drivers
task prelabRequest();
    // initialize packet
    /*
    pid_t pid = PID_OUT;
    pkt.pid = pid;
    pkt.addr = `DEVICE_ADDR;
    pkt.endp = `ADDR_ENDP;
    pkt.payload = 64'h23987;
    pkt_valid = 1'd1;
    #(400);*/
endtask : prelabRequest

  assign wires.DP = (sending | idle) ? wires_DP : 1'bz; // A tristate driver
  assign wires.DM = (sending | idle) ? wires_DM : 1'bz; // Another tristate driver
  assign wires_stateP = (wires.DP == 1'b1);
  assign wires_stateM = (wires.DM == 1'b1);


  logic NRZI_decode_bit_out; // bit from NRZI decode
  logic BS_decode_bit_out; // bit from BS decode
  logic send_inPID, send_inADDR, send_inENDP, send_inCRC;
  // what we're currently sending
  logic from_device_pkt_ready; // device sent a packet, decode should start
  logic CRC_decode_done; //tell NRZI decoder that we are done
  // BS decoder tells us a 0 bit stuffing will be sent next, skip it
  logic skip_0;
  // tell protocol handler what type of packet we are recieving
  logic recieve_ACK, recieve_NAC, recieve_DATA0, recieve_IN, recieve_OUT;
  logic corrupted; // the pkt has an error (CRC mismatch or pid mismatch)
  logic [4:0] CRC5_val_out; // CRC5 of the pkt being decoded
  logic [15:0] CRC16_val_out; // CRC16 of the pkt being decoded
  logic to_protocol_valid; // tell protocol handler we are done
  pkt_t decode_pkt; // pkt given from decoder to protocol handler
  logic got;
  // protocol handler got the packet decoder can stop holding on to it
  logic done_en;

  //handshaking between read/write and protocol fsm
  logic doneP, in, start, success, ended, re, doOp, successOut;
  logic [63:0] data0In, data0Out, dataIn, dataOut;
  logic [63:0] addr;
  logic decoding;
  logic firstOut;
  logic skip_0_2;

  //protocol handler
  protocolHandler p(.clock(clock), .reset(~reset_n), .encodeDone(done),
      .pktReady_enN(pkt_valid), .pkt_en(pkt),
      .pktReady_de(to_protocol_valid),
      .corr(corrupted), .pkt_de(decode_pkt), .got(got), .in(in),
      .start(start), .data0In(data0In),
      .data0Out(data0Out), .done(doneP), .success(success),.firstOut(firstOut));

  //read/write fsm
  readWrite r(.clock(clock), .reset(~reset_n), .re(re), .doOp(doOp),
      .mempage(addr), .dataIn(dataIn),.dataOut(dataOut),.successOut(successOut),
      .ended(ended), .done(doneP), .successIn(success), .dataPIn(data0Out),
      .dataPOut(data0In), .in(in), .start(start), .firstOut(firstOut));

   // CRC5_2_decode
  CRC5_2_decode m4(.clock(clock), .reset(~reset_n), .got(got),
                  .bit_in(BS_decode_bit_out), .pkt(decode_pkt),
                  .pkt_ready(from_device_pkt_ready), .skip_0(skip_0),
                  .done(CRC_decode_done),
                  .recieve_ACK(recieve_ACK), .recieve_NAC(recieve_NAC),
                  .recieve_DATA0(recieve_DATA0), .recieve_IN(recieve_IN),
                  .recieve_OUT(recieve_OUT), .corrupted(corrupted),
                  .send_inPID(send_inPID), .send_inADDR(send_inADDR),
                  .send_inENDP(send_inENDP),
                  .send_inCRC(send_inCRC) , .pkt_valid(to_protocol_valid),
                  .CRC5_val_out(CRC5_val_out), .CRC16_val_out(CRC16_val_out),
                  .skip_0_2(skip_0_2));
  // BS decoder
  BS_decode m5(.clock(clock), .reset(~reset_n), .bit_in(NRZI_decode_bit_out),
               .send_inPID(send_inPID), .send_inADDR(send_inADDR),
               .send_inENDP(send_inENDP),.send_inCRC(send_inCRC),
               .pkt_ready(from_device_pkt_ready), .skip_0(skip_0),
               .bit_out(BS_decode_bit_out) , .skip_0_2(skip_0_2));

  // NRZI decoder
  NRZI_decode m6(.clock(clock), .reset(~reset_n),
                 .pkt_ready(from_device_pkt_ready), .wires_stateP(wires_stateP),
                 .wires_stateM(wires_stateM), .sending(sending),
                 .bit_out(NRZI_decode_bit_out), .decoding(decoding));

  // CRC calculation
  CRC5_2 m1(.clock(clock), .reset(~reset_n), .pkt(pkt), .pkt_valid(pkt_valid),
            .send0(send0), .bit_out(CRC5_bit_out),.done(done),.done_en(done_en),
            .CRC5_val(CRC5_val), .sendPID(sendPID), .sendADDR(sendADDR),
            .sendENDP(sendENDP), .sendCRC(sendCRC), .sendPAYLOAD(sendPAYLOAD));

  // Bit stuffing
  BS m2(.clock(clock), .reset(~reset_n), .bit_in(CRC5_bit_out),
        .send0(send0), .bit_out(BS_bit_out), .sendPID(sendPID),
        .sendADDR(sendADDR), .sendENDP(sendENDP), .sendCRC(sendCRC),
        .sendPAYLOAD(sendPAYLOAD));

  // NRZI
  NRZI m3(.clock(clock), .reset(~reset_n), .bit_in(BS_bit_out),
          .usb_bus(usb_bus), .done(done_en), .pkt_valid(pkt_valid),
          .wires_DP(wires_DP), .wires_DM(wires_DM), .sending(sending),
          .decoding(decoding), .idle(idle));



task readData
// Host sends mempage to thumb drive using a READ (OUT->DATA0->IN->DATA0)
// transaction, and then receives data from it. This task should return both the
// data and the transaction status, successful or unsuccessful, to the caller.
( input logic [15:0] mempage, // Page to write
  output logic [63:0] data, // Vector of bytes to write
  output logic success);

  doOp <= 1'b0;
  addr <= {mempage, 48'b0};
  doOp <= 1'b1;
  re = 1'b1;
  wait(ended == 1'b1);
  doOp <= 1'b0;
  success <= successOut;
  data <= dataOut;

  #(1500);
  #(5);

endtask : readData

task writeData
// Host sends mempage to thumb drive using a WRITE (OUT->DATA0->OUT->DATA0)
// transaction, and then sends data to it. This task should return the
// transaction status, successful or unsuccessful, to the caller.
( input logic [15:0] mempage, // Page to write
  input logic [63:0] data, // Vector of bytes to write
  output logic success);

  doOp <= 1'b0;
  @(posedge clock);
  addr <= {mempage, 48'b0};
  doOp <= 1'b1;
  re <= 1'b0;
  dataIn <= data;

  wait (ended == 1'b1);
  success <= successOut;
  doOp <= 1'b0;

  #(100);
  #(5);

endtask : writeData

endmodule : USBHost

//calculate CRC5 while sneding bits
module CRC5_2 (
  input logic clock, reset,
  input pkt_t pkt,
  input logic pkt_valid,
  input logic send0,
  output logic sendPID,
  output logic sendADDR,
  output logic sendENDP,
  output logic sendCRC,
  output logic sendPAYLOAD,
  output logic bit_out,
  output logic done,
  output logic done_en,
  output logic [4:0] CRC5_val,
  output logic [15:0] CRC16_val);

  enum logic [4:0]{DATA,S3,S5,S6,S7,S8,S8_2, S8_3, CRC16, PAYLOAD, ENDP
        ,ADDR,START,SYNC,PID,CRC, EOP1, EOP2, EOP3, ACKSTATE} state, nextState;

  logic[6:0] ind;
  logic bit_in;
  logic [7:0] sync;
  logic [6:0] indr;
  logic [4:0] CRC_r;
  logic [15:0] CRC16_r;
  logic [7:0] pid_full;
  logic [6:0] new_ind;
  logic [6:0] new_indr;
  logic check;

   always_ff @ (posedge clock, posedge reset)

       if (reset) begin
          indr <= ind;
          CRC_r <= CRC5_val;
          CRC16_r <= CRC16_val;

          state <= START;
       end
       else begin
          CRC_r <= CRC5_val;
          CRC16_r <= CRC16_val;
          indr <= ind;
          state <= nextState;
      end

   always_comb begin
       case (state)

          // initialize values
          START: begin
            sendPID = 0;
            sendADDR = 0;
            sendENDP = 0;
            sendCRC = 0;
            sendPAYLOAD = 0;
            done_en = 0;
            sync = 8'b00000001;
            ind = 0;
            done = 0;
            CRC5_val = 5'b11111;
            CRC16_val = 16'hFFFF;
            if (pkt_valid) begin
              nextState = SYNC;
            end
            else begin
              nextState = START;
          end
          end

        // send out sync
           SYNC: begin
            bit_out = sync[7-indr]; // send MSB
            ind = indr + 1;
            if (indr < 7) begin
              nextState = SYNC;
            end
            else if (indr == 7) begin
              pid_full[3:0] = pkt.pid;
              pid_full[7:4] = ~pkt.pid;

              ind = 0;
              nextState = PID;
            end

           end
          // send out PID
           PID : begin
            sendPID = 1;
            sendADDR = 0;
            sendENDP = 0;
            sendCRC = 0;
            bit_out = pid_full[indr]; // send LSB
            ind = indr + 1;
            if (indr < 7) begin
              nextState = PID;
            end
            else if (indr == 7) begin
               ind = 0;

               if (pkt.pid == PID_ACK) begin
                nextState = ACKSTATE;
               end
               else if (pkt.pid == PID_NAK) begin
                nextState = ACKSTATE;
               end
               else if (pkt.pid == PID_DATA0) begin
                nextState = PAYLOAD;
               end
               else if (pkt.pid == PID_IN ) begin
                nextState = ADDR;
               end
                else if (pkt.pid == PID_OUT) begin

                nextState = ADDR;
               end
            end
           end

          // wait state for ACK and MAK
           ACKSTATE: begin
               done = 1'd1;
               done_en = 1'd1;
               nextState = START;
           end

          // check address now
           ADDR: begin
            sendPID = 0;
            sendADDR = 1;
            sendENDP = 0;
            sendCRC = 0;

            // combinational CRC calc
            bit_out = pkt.addr[indr]; // sned LSB first
            CRC5_val[0] = pkt.addr[indr] ^ CRC_r[4];
            CRC5_val[2] = (pkt.addr[indr]^ CRC_r[4]) ^ CRC_r[1];
            CRC5_val[4] = CRC_r[3];
            CRC5_val[3] = CRC_r[2];
            CRC5_val[1] = CRC_r[0];


            //  no send bit stuffing  and ind is below 6
            if (~send0 & indr < 6) begin
              ind = indr + 1;
              nextState = ADDR;
            end

             // no bit stuffing and ind is 6 -->
             //fnished checking address, go check endp
            else if (~send0 & indr == 6) begin
              // go check endp now
              ind = 0;
              nextState = ENDP;
            end

            // bit stuffing send 0 and ind is below 6
            else if (send0 & indr < 6 )begin
              nextState = S3;
            end
            // bit stuffing send 0 and next thing we check is endp field
            else begin
              ind = 0;
              nextState = S5;
            end


           end
           // (for checking address) bit stuffing is sending a 0 wait a cyle
           S3: begin
            nextState = ADDR;
           end

            // check endp now
           ENDP: begin
            sendPID = 0;
            sendADDR = 0;
            sendENDP = 1;
            sendCRC = 0;

            bit_out = pkt.endp[indr]; // send LSB first
            bit_in = pkt.endp[indr];
            CRC5_val[0] = pkt.endp[indr] ^ CRC_r[4];
            CRC5_val[2] = (pkt.endp[indr] ^ CRC_r[4]) ^ CRC_r[1];
            CRC5_val[4] = CRC_r[3];
            CRC5_val[3] = CRC_r[2];
            CRC5_val[1] = CRC_r[0];

            // reached end of endp field, now send CRC
            if (indr == 3 & ~send0) begin
              ind = 0;
              nextState = CRC;
            end
            else if (indr == 3 & send0) begin
              ind = 0;
              nextState = S6;
            end
            // else go ahead and check whether or not we wait or continue
            else if (send0) begin
              nextState = S5;
            end
            //
            else begin
              ind = indr + 1;
              nextState = ENDP;
            end

           end

            // (for checking endp) bit stuffing is sending a 0 wait a cyle
           S5: begin
            nextState = ENDP;
           end

          // send CRC
           CRC: begin
            sendPID = 0;
            sendADDR = 0;
            sendENDP = 0;
            sendCRC = 1;
            bit_out = ~CRC_r[4 - indr]; // complement and send as MSB

            // finsihed checking wait for new bits to be sent
            if (indr == 5) begin
              done_en = 1;
              nextState = EOP1;
            end

            else if (send0) begin
              nextState = S6;
            end

            // conitnue sending CRC5 encoding
            else begin
              ind =indr +  1;
              nextState = CRC;
            end
           end

           S6: begin
            nextState = CRC;
           end
          // we're sending a data packet
          PAYLOAD: begin
            sendPID = 0;
            sendADDR = 0;
            sendENDP = 0;
            sendCRC = 0;
            sendPAYLOAD = 1;
            done_en = 0;

            bit_out = pkt.payload[indr]; // send LSB first


              CRC16_val[0] = pkt.payload[indr] ^ CRC16_r[15];
              CRC16_val[2] = ( pkt.payload[indr] ^ CRC16_r[15]) ^CRC16_r[1] ;
              CRC16_val[15] = (pkt.payload[indr]^ CRC16_r[15]) ^ CRC16_r[14];
              CRC16_val[1] = CRC16_r[0];
              CRC16_val[3] = CRC16_r[2];
              CRC16_val[4] = CRC16_r[3];
              CRC16_val[5] = CRC16_r[4];
              CRC16_val[6] = CRC16_r[5];
              CRC16_val[7] = CRC16_r[6];
              CRC16_val[8] = CRC16_r[7];
              CRC16_val[9] = CRC16_r[8];
              CRC16_val[10] = CRC16_r[9];
              CRC16_val[11] = CRC16_r[10];
              CRC16_val[12] = CRC16_r[11];
              CRC16_val[13] = CRC16_r[12];
              CRC16_val[14] = CRC16_r[13];

            // reached end of data field, now send CRC16
            if (indr == 63 & ~send0) begin
              ind = 0;
              nextState = CRC16;
            end
            // go to wait state of CRC16
            else if (indr == 63 & send0) begin
              ind = 0;
              nextState = S8;
            end
            // if send0 go to wait state
            else if (send0) begin
              nextState = S7;
            end
            // otherwise keeping looping through
            else begin
              ind = indr + 1;
              nextState = PAYLOAD;
            end

           end

           // wait bit stuffing satet for payload
           S7: begin
            nextState = PAYLOAD;
           end

            // send CRC
           CRC16: begin
            done_en = 0;
            sendPID = 0;
            sendADDR = 0;
            sendENDP = 0;
            sendCRC = 1;

            if (indr != 16) begin
              bit_out = ~CRC16_r[15 - indr]; // complement and send as MSB
            end

            // finsihed checking wait for new bits to be sent

            // bit stuffing
            if (send0) begin
              done_en = 0;
              if (indr == 7'd16 )begin
                check = 1'd1;
                 done_en = 0;
                nextState = S8_2;
              end

              else if (indr == 7'd15 )begin
                check = 1'd0;
                nextState = S8;
              end
              else begin
                nextState = S8;
              end

            end

            // we've finished decoding the CRC
            else if (indr == 16) begin

              done_en = 1;
              nextState = EOP1;
            end

            // conitnue sending CRC16 encoding
            else begin
              ind =indr +  1;
              nextState = CRC16;
            end
           end

          // wait state for CRC16
           S8: begin
              nextState = CRC16;
           end

            S8_2: begin
              done_en = 1;
              nextState = S8_3;
           end

           S8_3 : begin
            done_en = 1;
            nextState = EOP1;

           end
           // wait for NRZI to send the EOP
           EOP1: begin
            nextState = EOP2;
           end

           EOP2: begin
            nextState = EOP3;
           end

           EOP3: begin
            done = 1;
            nextState = START;
           end

       endcase
   end

endmodule : CRC5_2

// bit stuffing FSM -- sequence detector of 6 ones
module BS (
  input logic clock, reset,
  input logic bit_in,
  input logic sendPID,
  input logic sendADDR,
  input logic sendENDP,
  input logic sendCRC,
  input logic sendPAYLOAD,
  output logic send0,
  output logic bit_out);

  enum logic [3:0] {S0, S1, S2, S3, S4, S5, S6, S7} state, nextState;
  logic ind;
   always_ff @ (posedge clock, posedge reset)
       if (reset)

           state <= S1;
       else
           state <= nextState;

   always_comb begin
       case (state)

          // only start counting if we're sending addr, endp, or crc
           S1: begin
            bit_out = bit_in;
            send0 = 0;
            if (sendADDR | sendENDP | sendCRC  | sendPAYLOAD) begin
              if (bit_in == 1'd1) begin
                nextState = S2;
              end

           end
            else begin
                nextState = S1;
              end
           end

           S2: begin
            bit_out = bit_in;
            if (bit_in == 1'd1) begin
              nextState = S3;
            end
            else begin
              nextState = S1;
            end

           end

           S3: begin
            bit_out = bit_in;
            if (bit_in == 1'd1) begin
              nextState = S4;
            end
            else begin
              nextState = S1;
            end

           end
           S4: begin
            bit_out = bit_in;
            if (bit_in == 1'd1) begin
              nextState = S5;
            end
            else begin
              nextState = S1;
            end

           end

           S5: begin
            bit_out = bit_in;
            if (bit_in == 1'd1) begin
              nextState = S6;
            end
            else begin
              nextState = S1;
            end

           end

           S6: begin
            bit_out = bit_in;
            if (bit_in == 1'd1) begin
              nextState = S7;
            end
            else begin
              nextState = S1;
            end

           end
          // counted 6 ones in a row, send a 0
           S7: begin
            bit_out = 1'd0;
            send0 = 1;
            nextState = S1;

           end


       endcase
   end

endmodule : BS

// NRZI FSM to drive the wires
module NRZI (
  input logic clock, reset,
  input logic bit_in,
  input logic done,
  input logic pkt_valid,
  input logic decoding,
  output logic idle,
  output logic sending,
  output logic wires_DM,
  output logic wires_DP,
  output usb_bus_t usb_bus);

  enum logic [3:0] {J,K, EOP, EOP1, EOP2, START, START2} state, nextState;

   always_ff @ (posedge clock, posedge reset)
       if (reset)
           state <= START;
       else
           state <= nextState;

   always_comb begin
       case (state)

       // Start assuming we've sent a J
          START: begin
            sending = 0;
            idle = 0;
            usb_bus = USB_X;
            if (pkt_valid) begin
              idle = 0;
              nextState = J;
            end
            else begin
              nextState = START;
              end

            end


          // send J
           J: begin
            sending = 1;
            wires_DP = 1'b1;
            wires_DM = 1'b0;
            if (done) begin
              nextState = EOP;
            end
            else if (bit_in == 1'd1) begin
              usb_bus = USB_J;
              nextState = J;
            end
            else begin
              if (bit_in == 1'd0) begin
              usb_bus = USB_K;
              nextState = K;
            end
            end

           end

          // send K
           K: begin
            wires_DP = 1'b0;
            wires_DM = 1'b1;
            if (done) begin
              nextState = EOP;
            end
            else if (bit_in == 1'd1) begin
              usb_bus = USB_K;
              nextState = K;
            end
            else begin
              if (bit_in == 1'd0) begin
              usb_bus = USB_J;
              nextState = J;
            end
            end
           end

           // sned out end sequence of packet (XXJ)
           EOP: begin
              wires_DP = 1'b0;
              wires_DM = 1'b0;
              usb_bus = USB_X;
              nextState = EOP1;
            end

             EOP1: begin
              wires_DP = 1'b0;
              wires_DM = 1'b0;
              usb_bus = USB_X;
              nextState = EOP2;
            end

             EOP2: begin
              wires_DP = 1'b1;
              wires_DM = 1'b0;
              usb_bus = USB_J;
              nextState = START;
            end

       endcase
   end

endmodule : NRZI


//CRC decoder
module CRC5_2_decode (
  input logic clock, reset,
  input logic got,
  input logic bit_in, // what we get from the BS_decoder
  output pkt_t pkt,
  input logic pkt_ready, // decoder is sending a packet
  input logic skip_0,
  input  logic skip_0_2,
  output logic done,
  output logic recieve_ACK,
  output logic recieve_NAC,
  output logic recieve_DATA0,
  output logic recieve_IN,
  output logic recieve_OUT,
  output logic corrupted,
  output logic send_inPID,
  output logic send_inADDR,
  output logic send_inENDP,
  output logic send_inCRC,
  output logic pkt_valid,
  output logic [4:0] CRC5_val_out,
  output logic [15:0] CRC16_val_out);

  enum logic [4:0]{PAYLOAD,S3,S5,S6,S7,S8,CRC16,ENDP,
                   ADDR,START,SYNC,PID,CRC} state, nextState;

  logic[5:0] ind;
  logic [7:0] sync;
  logic [5:0] indr;
  logic [4:0] CRC_r;
  logic [15:0] CRC16_r;
  logic [7:0] pid_full;
  logic [4:0 ] new_ind;
  logic [4:0] new_indr;
  logic [4:0] CRC_in;
  logic [15:0] CRC16_in;
  logic [15:0] check;
  logic check2; // for CRC calc when send 0

   always_ff @ (posedge clock, posedge reset)

       if (reset) begin
          indr <= ind;
          CRC_r <= CRC5_val_out;
          CRC16_r <= CRC16_val_out;

          state <= START;
       end
       else begin
          CRC_r <= CRC5_val_out;
          CRC16_r <= CRC16_val_out;
          check <= ~CRC16_val_out;
          indr <= ind;
          state <= nextState;
      end

   always_comb begin
       case (state)

          // initialize values
          START: begin
            recieve_ACK = 0;
            recieve_NAC = 0;
            recieve_DATA0 = 0;
            recieve_IN = 0;
            recieve_OUT = 0;
            pkt_valid = 0;
            corrupted = 0;
            send_inPID = 0;
            send_inADDR = 0;
            send_inENDP = 0;
            send_inCRC = 0;
            sync = 8'b00000001;
            ind = 0;
            done = 0;
            CRC5_val_out = 5'b11111;
            CRC16_val_out = 16'hFFFF;
            pkt.pid = 0;
            pkt.addr = 0;
            pkt.endp = 0;
            pkt.payload = 0;
            check2 = 0;
            if (pkt_ready) begin
              ind = 0; // start count of how many bits we're receiving
              nextState = PID;
            end
            else begin
              nextState = START;
          end
          end

            // check if the first half is inverted second half
           PID : begin

            send_inPID = 1;
            send_inADDR = 0;
            send_inENDP = 0;
            send_inCRC = 0;
            ind = indr + 1;

            if (indr < 8) begin
               pid_full[indr] = bit_in;
               nextState = PID;
            end

            // finsihed going through the PID bits
             if (indr == 8) begin
               ind = 0;
               pkt.pid = pid_full[3:0];
               if (pkt.pid == PID_ACK) begin
                recieve_ACK = 1'd1;
                pkt_valid = 1'd1;
                nextState = START;
               end
               else if (pkt.pid == PID_NAK) begin
                pkt_valid = 1'd1;
                recieve_NAC = 1'd1;
                nextState = START;
               end
               else if (pkt.pid == PID_DATA0) begin
                recieve_DATA0 = 1'd1;
                pkt.payload[ind] = bit_in;
                // start CRC calculation 
                CRC16_val_out[0] = bit_in ^ CRC16_r[15];
                CRC16_val_out[2] = (bit_in ^ CRC16_r[15]) ^CRC16_r[1] ;
                CRC16_val_out[15] = (bit_in^ CRC16_r[15]) ^ CRC16_r[14];
                CRC16_val_out[1] = CRC16_r[0];
                CRC16_val_out[3] = CRC16_r[2];
                CRC16_val_out[4] = CRC16_r[3];
                CRC16_val_out[5] = CRC16_r[4];
                CRC16_val_out[6] = CRC16_r[5];
                CRC16_val_out[7] = CRC16_r[6];
                CRC16_val_out[8] = CRC16_r[7];
                CRC16_val_out[9] = CRC16_r[8];
                CRC16_val_out[10] = CRC16_r[9];
                CRC16_val_out[11] = CRC16_r[10];
                CRC16_val_out[12] = CRC16_r[11];
                CRC16_val_out[13] = CRC16_r[12];
                CRC16_val_out[14] = CRC16_r[13];
                nextState = PAYLOAD;

               end
               else if (pkt.pid == PID_IN) begin
                recieve_IN = 1'd1;
                nextState = ADDR;
               end

                else if (pkt.pid == PID_OUT) begin
                recieve_OUT = 1'd1;
                nextState = ADDR;
               end
            end
           end

          // check address now
           ADDR: begin
            send_inPID = 0;
            send_inADDR = 1;
            send_inENDP = 0;
            send_inCRC = 0;

            pkt.addr[ind] = bit_in; // sent in, in LSB

            CRC5_val_out[0] = bit_in^ CRC_r[4];
            CRC5_val_out[2] = (bit_in^ CRC_r[4]) ^ CRC_r[1];
            CRC5_val_out[4] = CRC_r[3];
            CRC5_val_out[3] = CRC_r[2];
            CRC5_val_out[1] = CRC_r[0];

            //  no send bit stuffing  and ind is below 6
            if (~skip_0 & indr < 6) begin
              ind = indr + 1;
              nextState = ADDR;
            end

             // no bit stuffing and ind is 6 -->
             //fnished checking address, go check endp
            else if (~skip_0 & indr == 6) begin
              // go check endp now
              ind = 0;
              nextState = ENDP;
            end

            // bit stuffing send 0 and ind is below 6
            else if (skip_0 & indr < 6 )begin
              nextState = S3;
            end
            // bit stuffing send 0 and next thing we check is endp field
            else begin
              ind = 0;
              nextState = S5;
            end


           end
           // (for checking address) bit stuffing is sending a 0 wait a cyle
           S3: begin
            nextState = ADDR;
           end

            // check endp now
           ENDP: begin
            send_inPID = 0;
            send_inADDR = 0;
            send_inENDP = 1;
            send_inCRC = 0;

            pkt.endp[indr] = bit_in;

            CRC5_val_out[0] = pkt.endp[indr] ^ CRC_r[4];
            CRC5_val_out[2] = (pkt.endp[indr] ^ CRC_r[4]) ^ CRC_r[1];
            CRC5_val_out[4] = CRC_r[3];
            CRC5_val_out[3] = CRC_r[2];
            CRC5_val_out[1] = CRC_r[0];

            // reached end of endp field, now send CRC
            if (indr == 3 & ~skip_0) begin
              ind = 0;
              nextState = CRC;
            end
            else if (indr == 3 & skip_0) begin
              ind = 0;
              nextState = S6;
            end
            // else go ahead and check whether or not we wait or continue
            else if (skip_0) begin
              nextState = S5;
            end
            //
            else begin
              ind = indr + 1;
              nextState = ENDP;
            end

           end

           // (for checking endp) bit stuffing is sending a 0 wait a cyle
           S5: begin
            nextState = ENDP;
           end

           PAYLOAD: begin
            send_inPID = 0;
            send_inADDR = 0;
            send_inENDP = 0;
            send_inCRC = 1;


            nextState = S7;

            // if not bit stuffing
            // update increments and CRC calucaltion 
            if (~skip_0_2) begin
              ind = indr + 1;
              pkt.payload[ind] = bit_in;
              CRC16_val_out[0] = bit_in ^ CRC16_r[15];
              CRC16_val_out[2] = (bit_in ^ CRC16_r[15]) ^CRC16_r[1] ;
              CRC16_val_out[15] = (bit_in^ CRC16_r[15]) ^ CRC16_r[14];
              CRC16_val_out[1] = CRC16_r[0];
              CRC16_val_out[3] = CRC16_r[2];
              CRC16_val_out[4] = CRC16_r[3];
              CRC16_val_out[5] = CRC16_r[4];
              CRC16_val_out[6] = CRC16_r[5];
              CRC16_val_out[7] = CRC16_r[6];
              CRC16_val_out[8] = CRC16_r[7];
              CRC16_val_out[9] = CRC16_r[8];
              CRC16_val_out[10] = CRC16_r[9];
              CRC16_val_out[11] = CRC16_r[10];
              CRC16_val_out[12] = CRC16_r[11];
              CRC16_val_out[13] = CRC16_r[12];
              CRC16_val_out[14] = CRC16_r[13];
            end

            // reached end of endp field, now send CRC
            if (ind == 63 & ~skip_0_2) begin
              ind = 0;
              nextState = CRC16;
            end
            else if (ind == 63 & skip_0_2) begin
              ind = 0;
              nextState = S8;
            end
            // else go ahead and check whether or not we wait or continue
            else if (skip_0_2) begin
              nextState = S7;
            end

            else begin
              check2 = 0;
              nextState = PAYLOAD;
            end

           end
          
          // waitstate for payload 
           S7: begin
            nextState = PAYLOAD;
           end

           CRC16: begin
            send_inPID = 0;
            send_inADDR = 0;
            send_inENDP = 0;
            send_inCRC = 1;

            if (indr < 15) begin
              CRC16_in[15 - indr] = bit_in;
            end

            // finsihed checking wait for new bits to be sent
            if (indr == 16) begin

              if (CRC16_r != 16'h80_0d) begin
                corrupted = 1'd1;
              end
              else begin
                corrupted = 1'd0;
              end
              done = 1;
              pkt_valid = 1'd1;
              nextState = START;
            end

            else if (skip_0_2) begin
              nextState = S8;
            end

            else begin
              nextState = CRC16;
            end

            // conitnue sending CRC encoding
            if (~ (skip_0_2) ) begin
              CRC16_val_out[0] = bit_in ^ CRC16_r[15];
              CRC16_val_out[2] = (bit_in ^ CRC16_r[15]) ^CRC16_r[1] ;
              CRC16_val_out[15] = (bit_in^ CRC16_r[15]) ^ CRC16_r[14];
              CRC16_val_out[1] = CRC16_r[0];
              CRC16_val_out[3] = CRC16_r[2];
              CRC16_val_out[4] = CRC16_r[3];
              CRC16_val_out[5] = CRC16_r[4];
              CRC16_val_out[6] = CRC16_r[5];
              CRC16_val_out[7] = CRC16_r[6];
              CRC16_val_out[8] = CRC16_r[7];
              CRC16_val_out[9] = CRC16_r[8];
              CRC16_val_out[10] = CRC16_r[9];
              CRC16_val_out[11] = CRC16_r[10];
              CRC16_val_out[12] = CRC16_r[11];
              CRC16_val_out[13] = CRC16_r[12];
              CRC16_val_out[14] = CRC16_r[13];
              ind =indr +  1;
            end
            end

          // bit stuffing wait state for CRC16
           S8: begin
            nextState = CRC16;
          end

          // send CRC
           CRC: begin
            send_inPID = 0;
            send_inADDR = 0;
            send_inENDP = 0;
            send_inCRC = 1;
            // hold onto packet until protocol handler can use it
            if (pkt_valid) begin
              if (~got) begin
                nextState = CRC;
              end
              else begin
                nextState = START;
              end
            end
            CRC_in[4 - indr] = bit_in;

            // finsihed checking wait for new bits to be sent
            if (indr == 5) begin
              if (CRC5_val_out != 5'h0c) begin
                corrupted = 1'd1;
              end
              done = 1;
              pkt_valid = 1'd1;
              nextState = START;
              if (got) begin
                nextState = START;
              end
              else begin
                nextState = CRC;
              end
            end

            else if (skip_0) begin
              nextState = S6;
            end

            // conitnue sending CRC5 encoding
            else begin
            CRC5_val_out[0] = bit_in ^ CRC_r[4];
            CRC5_val_out[2] = (bit_in ^ CRC_r[4]) ^ CRC_r[1];
            CRC5_val_out[4] = CRC_r[3];
            CRC5_val_out[3] = CRC_r[2];
            CRC5_val_out[1] = CRC_r[0];
              ind =indr +  1;
              nextState = CRC;
            end

           end

            // wait satte for CRC
           S6: begin
            nextState = CRC;
          end

       endcase
   end

endmodule : CRC5_2_decode

// bit stuffing FSM decoder -- sequence detector
module BS_decode (
  input logic clock, reset,
  input logic bit_in, // bit from the NRZI decoder
  input logic send_inPID,
  input logic send_inADDR,
  input logic send_inENDP,
  input logic send_inCRC,
  input logic pkt_ready,
  output logic skip_0_2,
  output logic skip_0, // we have received 6 1 bits from the decoder, skip next
  output logic bit_out); // bit going to the CRC fsm

  enum logic [3:0] {S0, S1, S2, S3, S4, S5, S6, S7} state, nextState;
  logic ind;
   always_ff @ (posedge clock, posedge reset)
       if (reset)

           state <= S1;
       else
           state <= nextState;

   always_comb begin
       case (state)

          // only start counting if we're sending addr, endp, payload, or crc
           S1: begin
            skip_0 = 0;
            skip_0_2 = 0;
            if (pkt_ready) begin
              bit_out = bit_in;
              if (send_inADDR | send_inENDP | send_inCRC ) begin
                if (bit_in == 1'd1) begin
                  nextState = S2;
                end
                else begin
                  nextState = S1;
                end
            end
            end
            else begin
                nextState = S1;
              end
           end

           S2: begin
             skip_0_2 = 0;
            bit_out = bit_in;
            if (bit_in == 1'd1) begin
              nextState = S3;
            end
            else begin
              nextState = S1;
            end

           end

           S3: begin
             skip_0_2 = 0;
            bit_out = bit_in;
            if (bit_in == 1'd1) begin
              nextState = S4;
            end
            else begin
              nextState = S1;
            end

           end
           S4: begin
             skip_0_2 = 0;
            bit_out = bit_in;
            if (bit_in == 1'd1) begin
              nextState = S5;
            end
            else begin
              nextState = S1;
            end

           end

           S5: begin
            skip_0_2 = 0;
            bit_out = bit_in;
            if (bit_in == 1'd1) begin
              nextState = S6;
            end
            else begin
              nextState = S1;
            end

           end

           S6: begin
            bit_out = bit_in;

            if (bit_in == 1'd1) begin
               skip_0_2 = 1;
              nextState = S7;
            end
            else begin
              skip_0_2 = 0;
              nextState = S1;
            end

           end
          // counted 6 ones in a row, send a 0
           S7: begin
            bit_out = 1'd0;
            skip_0_2 = 0;
            skip_0 = 1;
            nextState = S1;

           end


       endcase
   end

endmodule : BS_decode


// NRZI FSM to decode the wires being drivin
module NRZI_decode (
  input logic clock, reset,
  //output logic done,
  output logic pkt_ready,  // decoder is starting to send bits
  input logic wires_stateP,
  input logic wires_stateM,
  input  logic sending,
  output logic decoding,
  output logic bit_out);

  enum logic [4:0] {J,K, EOP, EOP1, EOP2, START, SYNC1,
                    SYNC2,SYNC3, SYNC4, SYNC5, SYNC6, SYNC7} state, nextState;

   always_ff @ (posedge clock, posedge reset)
       if (reset)
           state <= START;
       else
           state <= nextState;

   always_comb begin
       case (state)

       // Start assuming we've sent a J
          START: begin
            decoding = 0;
            pkt_ready = 1'd0;
            if (~sending) begin // the wires are changing becuase of an input

              // should only be looking out for K
              if (~wires_stateP & wires_stateM) begin // if USB_K
                decoding = 1'b1;
                bit_out = 1'd0;
                nextState = SYNC1; // check for SYNC sequence
              end
              else begin
                nextState = START;
              end

            end
            else begin
              decoding = 1'b0;
              nextState = START;
            end
          end

          // send J
           J: begin
            if (wires_stateP & ~wires_stateM) begin  // USB_J
                bit_out = 1'd1;
                nextState = J;
              end
            if (~wires_stateP & wires_stateM) begin  // USB_K
              bit_out = 1'd0;
              nextState = K;
            end

            if (~wires_stateP & ~wires_stateM) begin // USB_X
              //done = 1;
              nextState = EOP;
            end
           end

          // we just came from sending a K
           K: begin
            if (wires_stateP & ~wires_stateM) begin // USB_J
                  bit_out = 1'd0;
                  nextState = J;
                end
            if (~wires_stateP & wires_stateM) begin // USB_K
              bit_out = 1'd1;
              nextState = K;
            end
            if (~wires_stateP & ~wires_stateM) begin // USB_X
               // done = 1;
                nextState = EOP;
              end
           end

           // sned out end sequence of packet (XXJ)
           EOP: begin
            pkt_ready = 1'd0;
            nextState = START;
            end

           SYNC1: begin
            if (wires_stateP & ~wires_stateM) begin // USB_J
                nextState = SYNC2; // check for SYNC sequence
              end
            else begin
              nextState = START; // missend
            end

           end
           SYNC2: begin
             if (~wires_stateP & wires_stateM) begin // USB_K
                nextState = SYNC3; // check for SYNC sequence
              end
            else begin
              nextState = START; // missend
            end

            end
           SYNC3: begin
             if (wires_stateP & ~wires_stateM) begin // USB_J
                nextState = SYNC4; // check for SYNC sequence
              end
            else begin
              nextState = START; // missend
            end

            end
           SYNC4: begin
            if (~wires_stateP & wires_stateM) begin // USB_K
                nextState = SYNC5; // check for SYNC sequence
              end
            else begin
              nextState = START; // missend
            end

            end
           SYNC5: begin
              if (wires_stateP & ~wires_stateM) begin // USB_J
                nextState = SYNC6; // check for SYNC sequence
              end
            else begin
              nextState = START; // missend
            end
            end
           SYNC6: begin
           if (~wires_stateP & wires_stateM) begin // USB_K
                nextState = SYNC7; // check for SYNC sequence
              end
            else begin
              nextState = START; // missend
            end

            end
           SYNC7: begin
             // USB_K , we have recived the full SYNC seqeunce
             if (~wires_stateP & wires_stateM) begin
                pkt_ready = 1'd1;
                // start to create the pkt from the bits that are sent in
                nextState = K;
              end
            else begin
              nextState = START; // missend
            end


            end


       endcase
   end

endmodule : NRZI_decode

//fsm for handling in/out protocols
module protocolHandler (
    input logic clock, reset,
    //handshaking signals to encoding
    input logic encodeDone, //pulse
    output logic pktReady_enN, //held
    output pkt_t pkt_en,
    //handshaking signals to decoding
    input logic pktReady_de, //pulse
    input logic corr,
    input pkt_t pkt_de,
    output logic got, //pulse
    //handshaking signals to read/write fsm
    input logic in,
    input logic start,
    input logic [63:0] data0In,
    input logic firstOut,
    output logic [63:0] data0Out,
    output logic done,
    output logic success);

  //send OUT pkt with addr
  task sendOut;
      pkt_en.pid = PID_OUT;
      pkt_en.addr = `DEVICE_ADDR;
      pkt_en.endp = `ADDR_ENDP;
      pkt_en.payload = '0;
  endtask: sendOut

  //send OUT pkt with data
  task sendOutData;
      pkt_en.pid = PID_OUT;
      pkt_en.addr = `DEVICE_ADDR;
      pkt_en.endp = `DATA_ENDP;
      pkt_en.payload = '0;
  endtask: sendOutData

  //send IN pkt
  task sendIn;
      pkt_en.pid = PID_IN;
      pkt_en.addr = `DEVICE_ADDR;
      pkt_en.endp = `DATA_ENDP;
      pkt_en.payload = '0;
  endtask: sendIn

  //send data pkt
  task sendData
      (input logic [63:0] data);
      pkt_en.pid = PID_DATA0;
      pkt_en.addr = `DEVICE_ADDR;
      pkt_en.endp = `DATA_ENDP;
      pkt_en.payload = data;
  endtask: sendData

  //send ack
  task sendAck;
      pkt_en.pid = PID_ACK;
      pkt_en.addr = `DEVICE_ADDR;
      pkt_en.endp = `DATA_ENDP;
      pkt_en.payload = '0;
  endtask: sendAck

  //send nak
  task sendNak;
      pkt_en.pid = PID_NAK;
      pkt_en.addr = `DEVICE_ADDR;
      pkt_en.endp = `DATA_ENDP;
      pkt_en.payload = '0;
  endtask: sendNak

  //cyc count
  logic [8:0] cyc, cycN;
  //encoder handshaking ready
  logic pktReady_en;
  //error counts
  logic [2:0] timeOutCount, corruptCount;
  logic [2:0] timeOutCountN, corruptCountN;

  enum logic [3:0] {IDLE, IN, TIMEOUTIN, CORRUPTIN, OUT1, OUT2,
      TIMEOUTOUT, CORRUPTOUT, SUCCESSIN, INNAKBUFFER, INBUF2} state, nextState;

  always_ff @(posedge clock, posedge reset) begin
      if (reset) begin
          state <= IDLE;
          cycN <= '0;
          timeOutCountN <= '0;
          corruptCountN <= '0;
          pktReady_enN <= '0;
      end
      else begin
          state <= nextState;
          cycN <= cyc;
          timeOutCountN <= timeOutCount;
          corruptCountN <= corruptCount;
          pktReady_enN <= pktReady_en;
      end
  end

  //encoding signal
  logic encoding;

  always_comb begin
      case (state)
          //idle wait until read/write signals start
          IDLE: begin
              //init vars
              data0Out = '0;
              done = 1'b0;
              success = 1'b0;
              pktReady_en = 1'b0;
              got = 1'b0;
              cyc = '0;
              corruptCount = '0;
              timeOutCount = '0;
              encoding = 1'b0;

              if (start && in) begin
                  sendIn;
                  pktReady_en = 1'b1;
                  nextState = IN;
              end
              else if (start && ~in && ~firstOut) begin
                  sendOutData;
                  pktReady_en = 1'b1;
                  nextState = OUT1;
              end
              else if (start && ~in && firstOut) begin
                  sendOut;
                  pktReady_en = 1'b1;
                  nextState = OUT1;
              end
              else nextState = IDLE;
          end

          //send IN pkt
          IN: begin
              //timeout takes prio
              pktReady_en = 1'b0;
              if (~encodeDone && ~encoding) begin
                  encoding = 1'b0;
                  nextState = IN;
                  pktReady_en = 1'b0;
              end
              else begin
                  encoding = 1'b1;
                  pktReady_en = 1'b0; //turn off when receive
                  if (cycN > 9'd255) begin
                      timeOutCount = timeOutCountN + 3'b1;
                      cyc = '0;
                      nextState = TIMEOUTIN;
                  end
                  else if (pktReady_de && ~corr) begin
                      cyc = '0;
                      done = 1'b1;
                      success = 1'b1;
                      data0Out = pkt_de.payload;
                      got = 1'b1;
                      nextState = SUCCESSIN;
                  end
                  else if (~pktReady_de) begin
                      cyc = cycN + 9'b1;
                      nextState = IN;
                  end
                  else if (pktReady_de && corr) begin
                      cyc = '0;
                      corruptCount = corruptCountN + 3'b1;
                      got = 1'b1;
                      nextState = CORRUPTIN;
                  end
              end
          end
          //if IN is success
          SUCCESSIN: begin
              nextState = IDLE;
              sendAck;
              pktReady_en = 1'b1;
          end
          //timeout on IN transaction
          TIMEOUTIN: begin
              if (timeOutCount < 'd7) begin
                  nextState = INNAKBUFFER;
              end
              else if (timeOutCount == 'd7) begin
                  success = 1'b0;
                  done = 1'b1;
                  nextState = IDLE;
              end
          end
          //corrupt on IN transaction
          CORRUPTIN: begin
              if (corruptCount < 'd7) begin
                  nextState = INNAKBUFFER;
              end
              else if (corruptCount == 'd7) begin
                  success = 1'b0;
                  done = 1'b1;
                  nextState = IDLE;
              end
          end
          //buffer for sending nak
          INNAKBUFFER: begin
              nextState = INBUF2;
          end

          //actually send nak
          INBUF2: begin
              nextState = IN;
              sendNak;
              pktReady_en = 1'b1;
          end
          //first out pkt (addr)
          OUT1: begin
              pktReady_en = 1'b0;
              if (~encodeDone) nextState = OUT1;
              else begin
                  nextState = OUT2;
                  pktReady_en = 1'b1;
                  sendData(data0In);
              end
          end
          //then send out (data)
          OUT2: begin
              if (~encodeDone && ~encoding) begin
                  encoding = 1'b0;
                  nextState = OUT2;
                  pktReady_en = 1'b1;
              end
              else begin
                  encoding = 1'b1;
                  pktReady_en = 1'b0;
                  if (cyc == 255) begin
                      timeOutCount = timeOutCountN + 3'b1;
                      cyc = '0;
                      nextState = TIMEOUTOUT;
                  end
                  else if (pktReady_de != 1'b1) begin
                      cyc = cycN + 9'b1;
                      nextState = OUT2;
                  end
                  else if (pktReady_de && pkt_de.pid == PID_NAK) begin
                      corruptCount = corruptCountN + 3'b1;
                      got = 1'b1;
                      nextState = CORRUPTOUT;
                  end
                  else if (pktReady_de && pkt_de.pid == PID_ACK) begin
                      success = 1'b1;
                      done = 1'b1;
                      got = 1'b1;
                      nextState = IDLE;
                  end
              end
          end
          //timeout on OUT
          TIMEOUTOUT: begin
              if (timeOutCount < 'd7) begin
                  sendData(data0In);
                  pktReady_en = 1'b1;
                  nextState = OUT2;
              end
              else if (timeOutCount == 'd7) begin
                  success = 1'b0;
                  done = 1'b1;
                  nextState = IDLE;
              end
          end
          //corrupt on OUT
          CORRUPTOUT: begin
              if (corruptCount < 'd7) begin
                  sendData(data0In);
                  pktReady_en = 1'b1;
                  nextState = OUT2;
              end
              else if (corruptCount == 'd7) begin
                  success = 1'b0;
                  done = 1'b1;
                  nextState = IDLE;
              end
          end
      endcase
  end
endmodule

//read/write fsm for handling task calls
module readWrite (
    input logic clock, reset,
    //task call
    input logic re, doOp,
    input logic [63:0] mempage,
    input logic [63:0] dataIn,
    output logic [63:0] dataOut,
    output logic successOut,
    output logic ended,
    //protocol
    input logic done, successIn,
    input logic [63:0] dataPIn,
    output logic [63:0] dataPOut,
    output logic firstOut,
    output logic in, start);

  enum logic [3:0] {IDLE, WAIT, NEXTIN, NEXTOUT} state, nextState;

  always_ff @(posedge clock, posedge reset) begin
      if (reset) begin
          state <= IDLE;
      end
      else state <= nextState;
  end

  always_comb begin

      case (state)
          IDLE: begin
              dataOut = '0;
              successOut = '0;
              ended = '0;
              dataPOut = '0;
              firstOut = '0;
              in = '0;
              start = '0;
              if (doOp == 1'b1) begin
                  start = 1'b1;
                  in = 1'b0;
                  dataPOut = mempage;
                  nextState = WAIT;
                  firstOut = 1'b1;
              end
              else nextState = IDLE;
          end
          //first out for addr
          WAIT: begin
              start = 1'b1;
              if (~done) nextState = WAIT;
              else if (done && ~successIn) begin
                  ended = 1'b1;
                  successOut = 1'b0;
                  nextState = IDLE;
              end
              //in op
              else if (done && successIn && re) begin
                  start = 1'b1;
                  in = 1'b1;
                  firstOut = 1'b0;
                  nextState = NEXTIN;
              end
              //out op
              else if (done && successIn && ~re) begin
                  start = 1'b1;
                  in = 1'b0;
                  dataPOut = dataIn;
                  firstOut = 1'b0;
                  nextState = NEXTOUT;
              end
          end
          NEXTIN: begin
              if (~done) nextState = NEXTIN;
              else if (done && successIn) begin
                  start = 1'b0;
                  successOut = 1'b1;
                  dataOut = dataPIn;
                  ended = 1'b1;
                  nextState = IDLE;
              end
              else if (done && ~successIn) begin
                  start = 1'b0;
                  ended = 1'b1;
                  successOut = 1'b0;
                  nextState = IDLE;
              end
          end
          NEXTOUT: begin
              if (~done) nextState = NEXTOUT;
              else if (done && successIn) begin
                  successOut = 1'b1;
                  ended = 1'b1;
                  nextState = IDLE;
                  start = 1'b0;
              end
              else if (done && ~successIn) begin
                  ended = 1'b1;
                  successOut = 1'b0;
                  nextState = IDLE;
                  start = 1'b0;
              end
          end
      endcase
  end
endmodule: readWrite
