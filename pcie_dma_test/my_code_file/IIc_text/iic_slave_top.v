//测试简单的数据交互
//可在i2cSlave_define.v中修改IIC地址
//在registerInterface.v中修改寄存器
module iic_slave_top
(
    input    wire            sys_clk    ,   //系统时钟 25MHZ

	input	 wire		     i2c_scl    ,	// IIC 时钟线
	inout 	 wire		     i2c_sda    ,	// IIC 数据线
	output	 wire    [1:0]	 led            // 用户LED

);
//IIC register                                   
wire  [7:0]led_reg;//0x01
wire  [7:0] myReg0_flag;

wire    clk_100m    ;
wire    rst_n       ;
reg   Reg0_wr_en;
reg   Reg1_wr_en;
reg   [7:0]  myReg0_w;
reg   [7:0]  myReg1_w;
//生成100MHZ的时钟
iic_pll iic_pll_inst (
  .clkout0(clk_100m),    // output
  .lock(rst_n),          // output
  .clkin1(sys_clk)       // input
);
//iic slave模块
i2cSlave i2cSlave_u (
	.clk		(clk_100m),		
	.rst		(~rst_n  ),		
	.sda		(i2c_sda ),		
	.scl		(i2c_scl ),	
    .Reg0_wr_en (Reg0_wr_en ),		
    .Reg1_wr_en (Reg1_wr_en ),		
    .myReg0_w   (myReg0_w ),		
	.myReg1_w   (myReg1_w ),		
	.myReg0		(myReg0_flag ),		
	.myReg1		(led_reg )		
);
//来自01寄存器的低2bit
assign	led 	= led_reg[1:0];




endmodule