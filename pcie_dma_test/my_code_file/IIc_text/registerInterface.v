//////////////////////////////////////////////////////////////////////
////                                                              ////
//// registerInterface.v                                          ////
////                                                              ////
//// This file is part of the i2cSlave opencores effort.
////                                                              ////
//// Module Description:                                          ////
//// You will need to modify this file to implement your
//// interface.
//// Add your control and status bytes/bits to module inputs and outputs,
//// and also to the I2C read and write process blocks
////                                                              ////
//// To Do:                                                       ////
////
////                                                              ////
//// Author(s):                                                   ////
//// - Steve Fielding, sfielding@base2designs.com                 ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE. See the GNU Lesser General Public License for more  ////
//// details.                                                     ////
////                                                              ////                                                            
//////////////////////////////////////////////////////////////////////
//
`include "i2cSlave_define.v"

module registerInterface (
  rst,
  clk,
  addr,
  dataIn,
  writeEn,
  dataOut,
  Reg0_wr_en,		
  Reg1_wr_en,			
  myReg0_w,
  myReg1_w,
  myReg0,
  myReg1,
  myReg2
);
input rst;
input clk;
input [7:0] addr;
input [7:0] dataIn;
input writeEn;
output [7:0] dataOut;
output [7:0] myReg0;
output [7:0] myReg1;
input [7:0] myReg2;
input   Reg0_wr_en;		
input   Reg1_wr_en;
input [7:0] myReg0_w;
input [7:0] myReg1_w;
//input fpga_myReg0_w_en;
reg [7:0] dataOut;
reg [7:0] myReg0;
reg [7:0] myReg1;

// --- I2C Read
always @(posedge clk) begin
  case (addr)
    8'h00: dataOut <= myReg0;
    8'h01: dataOut <= myReg1;
    8'h02: dataOut <= myReg2;  //only read
    default: dataOut <= 8'h00;
  endcase
end

// --- I2C Write
always @(posedge clk or posedge rst) begin
 if(rst)
   begin
    	myReg0<=0;
		myReg1<=0;
   
  end
else
  if (writeEn == 1'b1) begin
    case (addr)
      8'h00: myReg0 <= dataIn;
      8'h01: myReg1 <= dataIn;
    endcase
  end
  else begin
      if (Reg0_wr_en == 1'b1) begin
         myReg0 <= myReg0_w;
      end
      if (Reg1_wr_en == 1'b1) begin
         myReg1 <= myReg1_w;
      end
  end
end

endmodule



