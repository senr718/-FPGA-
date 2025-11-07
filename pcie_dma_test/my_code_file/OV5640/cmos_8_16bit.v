`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:Meyesemi 
// Engineer: Will
// 
// Create Date: 2023-03-17  
// Module Name: cmos_8_16bit
// Description: 将两个8bit数据拼成一个16bit RGB565数据（替换GTP模块为普通逻辑）
// Target Devices: Pango
//////////////////////////////////////////////////////////////////////////////////

module cmos_8_16bit(
	input 				   pclk 		,   
	input 				   rst_n		,
	input				   de_i	        ,
	input	[7:0]	       pdata_i	    ,
    input                  vs_i         ,

    output                 pixel_clk    ,
 	output	reg			   de_o         ,
	output  reg [15:0]	   pdata_o
); 
reg			de_out1          ;
reg [15:0]	pdata_out1       ;
reg			de_out2          ;
reg [15:0]	pdata_out2       ;    
reg [1:0]   cnt             ;
// --------------- 替换GTP模块：用普通信号替代pclk_IOCLKBUF ---------------
wire        pclk_IOCLKBUF   ;  // 原GTP缓冲后时钟，现在用普通逻辑生成
reg         vs_i_reg        ;
reg         enble           ;
reg [7:0] pdata_i_reg;
reg de_i_r,de_i_r1;
reg			de_out3          ;
reg [15:0]	pdata_out3       ;  
// --------------- 新增：2分频寄存器（替代GTP_IOCLKDIV_E2） ---------------
reg         pclk_div2       ;  // 用于实现2分频的寄存器


// ---------------------------- 原逻辑保留：场同步使能 ----------------------------
always @(posedge pclk)begin
       vs_i_reg <= vs_i ;
end

always@(posedge pclk)
    begin
        if(!rst_n)
            enble <= 1'b0;
        else if(!vs_i_reg&&vs_i)
            enble <= 1'b1;
        else
            enble <= enble;
    end


// ---------------------------- 替换1：GTP_IOCLKBUF（时钟使能缓冲） ----------------------------
// 功能：enble=1时传递pclk（模拟缓冲），enble=0时阻断时钟（避免无效时钟）
assign pclk_IOCLKBUF = (enble == 1'b1) ? pclk : 1'b0;


// ---------------------------- 替换2：GTP_IOCLKDIV_E2（2分频） ----------------------------
// 功能：对pclk_IOCLKBUF进行2分频，生成pixel_clk（CE=1'b1、RST_N=enble保持原逻辑）
always @(posedge pclk_IOCLKBUF or negedge enble) begin  // 复位用enble（低有效）
    if(!enble) begin  // 原RST_N=enble，低有效复位
        pclk_div2 <= 1'b0;
    end else begin  // 原CE=1'b1，始终允许分频
        pclk_div2 <= ~pclk_div2;  // 时钟上升沿翻转，实现2分频
    end
end
assign pixel_clk = pclk_div2;  // 分频后时钟输出（替代原CLKDIVOUT）


// ---------------------------- 原逻辑完全保留：数据拼接与同步 ----------------------------
always@(posedge pclk)
    begin
        if(!rst_n)
            cnt <= 2'b0;
        else if(de_i == 1'b1 && cnt == 2'd1)
            cnt <= 2'b0;
        else if(de_i == 1'b1)
            cnt <= cnt + 1'b1;
    end

always@(posedge pclk)
    begin
        if(!rst_n)
            pdata_i_reg <= 8'b0;
        else if(de_i == 1'b1)
            pdata_i_reg <= pdata_i;
    end

always@(posedge pclk)
    begin
        if(!rst_n)
            pdata_out1 <= 16'b0;
        else if(de_i == 1'b1 && cnt == 2'd1)
            pdata_out1 <= {pdata_i_reg,pdata_i};
    end

always@(posedge pclk)begin
    de_i_r <= de_i;
    de_i_r1 <= de_i_r;
end

always@(posedge pclk)
    begin
        if(!rst_n)
            de_out1 <= 1'b0;
        else if(!de_i_r1 && de_i_r )//de_i上升沿
            de_out1 <= 1'b1;
        else if(de_i_r1 && !de_i_r )//de_i下降沿
            de_out1 <= 1'b0;
        else
            de_out1 <= de_out1;
    end

always@(posedge pixel_clk)begin
    de_out2<=de_out1;
    de_out3<=de_out2;
    de_o   <=de_out3;
end

always@(posedge pixel_clk)begin
    pdata_out2<=pdata_out1;
    pdata_out3<=pdata_out2;
    pdata_o   <=pdata_out3;
end

endmodule