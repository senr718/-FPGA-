//pango参考例程
//基于官方demo修改
`timescale 1ns / 1ps

`define UD #1
//cmos1、cmos2二选一，作为视频源输入
`define CMOS_1      //cmos1作为视频输入；
`define CMOS_2      //cmos2作为视频输入；
module pcie_dma_test #(
  parameter VIDEO_LENGTH         = 1920                 ,
  parameter VIDEO_HIGTH          = 1080                 ,
  parameter ZOOM_VIDEO_LENGTH    = 960                 ,
  parameter ZOOM_VIDEO_HIGTH     = 540                 ,
  parameter PIXEL_WIDTH          = 32                 ,    
  parameter MEM_ROW_ADDR_WIDTH   = 15                 ,
  parameter MEM_COL_ADDR_WIDTH   = 10                 ,
  parameter MEM_BADDR_WIDTH      = 3                  ,
  parameter M_AXI_BRUST_LEN      = 8                  ,
  parameter RW_ADDR_MIN          = 20'b0              ,
  parameter RW_ADDR_MAX          = ZOOM_VIDEO_LENGTH*ZOOM_VIDEO_HIGTH*PIXEL_WIDTH/16       //@540p  518400个地址   
     

 )(

	// LED signals
	output reg				ref_led,    	
	output reg				pclk_led,
	input		[1:0]		rxn,			
	input		[1:0]		rxp,			
	output wire	[1:0]		txn,			
	output wire	[1:0]		txp,
	input					button_rst_n,	
	input					ref_clk_p,		
	input					ref_clk_n,		
	input					perst_n,		
			

    input    wire            sys_clk    ,   //系统时钟 25MHZ

	input	 wire		     i2c_scl    ,	// IIC 时钟线
	inout 	 wire		     i2c_sda    ,	// IIC 数据线

    output                               ddr_mem_cs_n                  , 
    output                               ddr_mem_rst_n                 ,
    output                               ddr_mem_ck                    ,
    output                               ddr_mem_ck_n                  ,
    output                               ddr_mem_cke                   ,
    output                               ddr_mem_ras_n                 ,
    output                               ddr_mem_cas_n                 ,
    output                               ddr_mem_we_n                  ,
    output                               ddr_mem_odt                   ,
    output      [14:0]                   ddr_mem_a                     ,
    output      [2:0]                    ddr_mem_ba                    ,
    inout       [1:0]                    ddr_mem_dqs                   ,
    inout       [1:0]                    ddr_mem_dqs_n                 ,
    inout       [15:0]                   ddr_mem_dq                    ,
    output      [1:0]                    ddr_mem_dm                    ,
		

//OV5647
    //output  [1:0]                        cmos_init_done       ,//OV5640寄存器初始化完成
    //coms1	
    inout                                cmos1_scl            ,//cmos1 i2c 
    inout                                cmos1_sda            ,//cmos1 i2c 
    input                                cmos1_vsync          ,//cmos1 vsync
    input                                cmos1_href           ,//cmos1 hsync refrence,data valid
    input                                cmos1_pclk           ,//cmos1 pxiel clock
    input   [7:0]                        cmos1_data           ,//cmos1 data
    output                               cmos1_reset          ,//cmos1 reset
    //coms2
    inout                                cmos2_scl            ,//cmos2 i2c 
    inout                                cmos2_sda            ,//cmos2 i2c 
    input                                cmos2_vsync          ,//cmos2 vsync
    input                                cmos2_href           ,//cmos2 hsync refrence,data valid
    input                                cmos2_pclk           ,//cmos2 pxiel clock
    input   [7:0]                        cmos2_data           ,//cmos2 data
    output                               cmos2_reset          , //cmos2 reset
   
    inout                                cmos3_scl            ,//cmos2 i2c 
    inout                                cmos3_sda            ,//cmos2 i2c 
    input                                cmos3_vsync          ,//cmos2 vsync
    input                                cmos3_href           ,//cmos2 hsync refrence,data valid
    input                                cmos3_pclk           ,//cmos2 pxiel clock
    input   [7:0]                        cmos3_data           ,//cmos2 data
    output                               cmos3_reset          , //cmos2 reset


       //ETH0_RGMII 
    output wire                                  eth_rst_n_0        , //以太网复位信号
    input  wire                                  eth_rgmii_rxc_0    ,
    input  wire                                  eth_rgmii_rx_ctl_0 ,
    input  wire [3:0]                            eth_rgmii_rxd_0    ,  
                       
    output wire                                  eth_rgmii_txc_0    ,
    output wire                                  eth_rgmii_tx_ctl_0 ,
    output wire [3:0]                            eth_rgmii_txd_0   

);

//开发板MAC地址 00-11-22-33-44-55
parameter  BOARD_MAC = 48'h00_11_22_33_44_55;     
//开发板IP地址 192.168.1.10     
parameter  BOARD_IP  = {8'd192,8'd168,8'd1,8'd10};
//目的MAC地址 ff_ff_ff_ff_ff_ff
parameter  DES_MAC   = 48'h58_11_22_91_38_31;


// 新增：全局复位信号（同步button_rst_n并结合PLL锁定）
wire global_rst_n;

assign global_rst_n = sync_button_rst_n  & ddr_init_done ; // 观察规律

///////////////////////////////////摄像头/////////////////////////////////////////


    reg                       zoom_vs_in_d0   ;
    reg                       zoom_vs_in_d1   ;
    reg                       zoom_de_in_d0   ;
    reg                       zoom_de_in_d1   ;
    reg [11 : 0]              zoom_de_in_cnt  /* synthesis PAP_MARK_DEBUG="1" */;
    reg [3 : 0]               zoom_de_in_state;

    reg                       video0_rd_en/* synthesis PAP_MARK_DEBUG="1" */;
    reg                       video1_rd_en/* synthesis PAP_MARK_DEBUG="1" */;
    reg                       video2_rd_en/* synthesis PAP_MARK_DEBUG="1" */;
    reg                       video3_rd_en/* synthesis PAP_MARK_DEBUG="1" */;
    reg                       video_pre_rd_flag/* synthesis PAP_MARK_DEBUG="1" */;
    wire                      w_video_pre_rd_flag;
    assign w_video_pre_rd_flag = video_pre_rd_flag;
    reg                        v_sync_flag;


    wire                          cmos_init_done1       ;//OV5640寄存器初始化完成
    wire                          cmos_init_done2       ;//OV5640寄存器初始化完成
    parameter TH_1S = 27'd33000000;
/////////////////////////////////////////////////////////////////////////////////////
    reg  [16:0]                 rstn_1ms            ;
    wire                        cmos_scl            ;//cmos i2c clock
    wire                        cmos_sda            ;//cmos i2c data
    wire                        cmos_vsync          ;//cmos vsync
    wire                        cmos_href           ;//cmos hsync refrence,data valid
    wire                        cmos_pclk           ;//cmos pxiel clock
    wire   [7:0]                cmos_data           ;//cmos data
    wire                        cmos_reset          ;//cmos reset
    wire                        initial_en          ;
    wire[15:0]                  cmos1_d_16bit       /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos1_href_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
    reg [7:0]                   cmos1_d_d0          /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos1_href_d0       /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos1_vsync_d0      /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos1_pclk_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
    wire[15:0]                  cmos2_d_16bit       /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos2_href_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
    reg [7:0]                   cmos2_d_d0          /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos2_href_d0       /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos2_vsync_d0      /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos2_pclk_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
    wire[15:0]                  cmos3_d_16bit       /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos3_href_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
    reg [7:0]                   cmos3_d_d0          /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos3_href_d0       /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos3_vsync_d0      /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos3_pclk_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
    wire[15:0]                  o_rgb565            ;
    wire                        pclk_in_test        ;    
    wire                        vs_in_test          ;
    wire                        de_in_test          ;
    wire[15:0]                  i_rgb565            ;
    wire                        pclk_in_test2        ;    
    wire                        vs_in_test2       ;
    wire                        de_in_test2          ;
    wire[15:0]                  i2_rgb565            ;
    wire                        pclk_in_test3        ;    
    wire                        vs_in_test3          ;
    wire                        de_in_test3          ;
    wire[15:0]                  i_rgb5653            ;
    wire                        de_re               ;

    reg  [26:0]                 cnt                        ;
    reg  [15:0]                 cnt_1                      ;




    always @(posedge clk_100m)
    begin
    	if(!rst_n)
    	    rstn_1ms <= 17'd0;
    	else
    	begin
    		if(rstn_1ms == 17'h186A0)
    		    rstn_1ms <= rstn_1ms;
    		else
    		    rstn_1ms <= rstn_1ms + 1'b1;
    	end
    end
    
    assign rstn_out = (rstn_1ms == 17'h186A0);


/////////////////////////////////////////////////////////////////////////////////////


    PLL my_PLL (
  .clkout0(clk_100m),    // output 100MHz
  .clkout1(clk_50M),    // output 50MHZ
  .lock(rst_n),          // output
  .clkin1(sys_clk)       // input 25M
);




//配置CMOS///////////////////////////////////////////////////////////////////////////////////
//OV5640 register configure enable    
    power_on_delay	power_on_delay_inst(
    	.clk_50M                 (clk_50M        ),//input
    	//.reset_n                 (1'b1           ),//input	
        .reset_n                 (global_rst_n   ),//input	
    	.camera1_rstn            (cmos1_reset    ),//output
    	.camera2_rstn            (cmos2_reset    ),//output	
    	.camera_pwnd             (               ),//output
    	.initial_en              (initial_en     ) //output		
    );
//CMOS1 Camera 
    reg_config	coms1_reg_config(
    	.clk_25M                 (sys_clk            ),//input
    	.camera_rstn             (global_rst_n & cmos1_reset        ),//input
    	.initial_en              (initial_en         ),//input		
    	.i2c_sclk                (cmos1_scl          ),//output
    	.i2c_sdat                (cmos1_sda          ),//inout
    	.reg_conf_done           (cmos_init_done1  ),//output config_finished
    	.reg_index               (                   ),//output reg [8:0]
    	.clock_20k               (                   ) //output reg
    );

//CMOS2 Camera 
    reg_config	coms2_reg_config(
    	.clk_25M                 (sys_clk            ),//input
    	.camera_rstn             (global_rst_n & cmos2_reset        ),//input
    	.initial_en              (initial_en         ),//input		
    	.i2c_sclk                (cmos2_scl          ),//output
    	.i2c_sdat                (cmos2_sda          ),//inout
    	.reg_conf_done           (cmos_init_done2  ),//output config_finished
    	.reg_index               (                   ),//output reg [8:0]
    	.clock_20k               (                   ) //output reg
    );
//CMOS 8bit转16bit///////////////////////////////////////////////////////////////////////////////////

//CMOS1
    always@(posedge cmos1_pclk)
        begin
            cmos1_d_d0        <= cmos1_data    ;
            cmos1_href_d0     <= cmos1_href    ;
            cmos1_vsync_d0    <= cmos1_vsync   ;
        end

    cmos_8_16bit cmos1_8_16bit(
    	.pclk           (cmos1_pclk       ),//input
    	.rst_n          (cmos_init_done1),//input
    	.pdata_i        (cmos1_d_d0       ),//input[7:0]
    	.de_i           (cmos1_href_d0    ),//input
    	.vs_i           (cmos1_vsync_d0    ),//input
    	
    	.pixel_clk      (cmos1_pclk_16bit ),//output
    	.pdata_o        (cmos1_d_16bit    ),//output[15:0]
    	.de_o           (cmos1_href_16bit ) //output
    );
//CMOS2
    always@(posedge cmos2_pclk)
        begin
            cmos2_d_d0        <= cmos2_data    ;
            cmos2_href_d0     <= cmos2_href    ;
            cmos2_vsync_d0    <= cmos2_vsync   ;
        end

    cmos_8_16bit cmos2_8_16bit(
    	.pclk           (cmos2_pclk       ),//input
    	.rst_n          (cmos_init_done2),//input
    	.pdata_i        (cmos2_d_d0       ),//input[7:0]
    	.de_i           (cmos2_href_d0    ),//input
    	.vs_i           (cmos2_vsync_d0    ),//input
    	
    	.pixel_clk      (cmos2_pclk_16bit ),//output
    	.pdata_o        (cmos2_d_16bit    ),//output[15:0]
    	.de_o           (cmos2_href_16bit ) //output
    );
//输入视频源选择//////////////////////////////////////////////////////////////////////////////////////////

assign     pclk_in_test    =    cmos1_pclk_16bit    ;
assign     vs_in_test      =    cmos1_vsync_d0      ;
assign     de_in_test      =    cmos1_href_16bit    ;
assign     i_rgb565        =    {cmos1_d_16bit[4:0],cmos1_d_16bit[10:5],cmos1_d_16bit[15:11]};//{r,g,b}

assign     pclk_in_test2    =    cmos2_pclk_16bit    ;
assign     vs_in_test2      =    cmos2_vsync_d0      ;
assign     de_in_test2      =    cmos2_href_16bit    ;
assign     i_rgb5652        =    {cmos2_d_16bit[4:0],cmos2_d_16bit[10:5],cmos2_d_16bit[15:11]};//{r,g,b}



// ========================== 1. 新增：核心参数与信号声明（基于指定文件） ==========================
// DDR3参数（来自DDR3_IP_源码.docx的ddr3模块）
localparam MEM_ROW_WIDTH    = 15          ;// 行地址宽度
localparam MEM_COLUMN_WIDTH = 10          ;// 列地址宽度
localparam MEM_BANK_WIDTH   = 3           ;// Bank宽度（8个Bank）
localparam MEM_DQ_WIDTH     = 16          ;// DQ位宽（16bit）
localparam MEM_DM_WIDTH     = MEM_DQ_WIDTH/8 ;// DM位宽（2bit）
localparam MEM_DQS_WIDTH    = MEM_DQ_WIDTH/8 ;// DQS位宽（2bit）
localparam CTRL_ADDR_WIDTH  = MEM_ROW_WIDTH + MEM_COLUMN_WIDTH + MEM_BANK_WIDTH ;// DDR3总地址宽度（28bit）
// RGB888相关参数（来自用户需求+摄像头例程）
localparam FIFO_WIDTH       = 128          ;// FIFO宽度（24bit RGB888+8bit填充）
// DDR3 IP输出信号
wire                        ddr_core_clk  /*synthesis PAP_MARK_DEBUG="1"*/;// DDR3核心时钟
wire                        ddr_init_done/*synthesis PAP_MARK_DEBUG="1"*/; // DDR3初始化完成
wire                        ddr_mem_cs_n  ;// DDR3片选
wire                        ddr_mem_rst_n ;// DDR3复位输出
wire                        ddr_mem_ck    ;// DDR3时钟
wire                        ddr_mem_ck_n  ;// DDR3时钟负
wire                        ddr_mem_cke   ;// DDR3时钟使能
wire                        ddr_mem_ras_n ;// DDR3 RAS
wire                        ddr_mem_cas_n ;// DDR3 CAS
wire                        ddr_mem_we_n  ;// DDR3 WE
wire                        ddr_mem_odt   ;// DDR3 ODT
wire [14:0]                 ddr_mem_a     ;// DDR3地址线
wire [2:0]                  ddr_mem_ba    ;// DDR3 Bank线
//inout [MEM_DQS_WIDTH-1:0]   ddr_mem_dqs   ;// DDR3 DQS
//inout [MEM_DQS_WIDTH-1:0]   ddr_mem_dqs_n ;// DDR3 DQS负
//inout [MEM_DQ_WIDTH-1:0]    ddr_mem_dq     ;// DDR3 DQ
wire [1:0]                  ddr_mem_dm     ;// DDR3 DM
// DDR3 AXI读写信号
wire [CTRL_ADDR_WIDTH-1:0]  ddr_axi_awaddr;// 写地址
wire [3:0]                  ddr_axi_awlen  ;// 写Burst长度
wire                        ddr_axi_awready/*synthesis PAP_MARK_DEBUG="1"*/;// 写地址准备好
wire                        ddr_axi_awvalid/*synthesis PAP_MARK_DEBUG="1"*/;// 写地址有效
wire [8*MEM_DQ_WIDTH-1:0]   ddr_axi_wdata  ;// 写数据（128bit）
wire [MEM_DQ_WIDTH-1:0]     ddr_axi_wstrb  ;// 写字节使能
wire                        ddr_axi_wready /*synthesis PAP_MARK_DEBUG="1"*/;// 写数据准备好
wire                        ddr_axi_wusero_last /*synthesis PAP_MARK_DEBUG="1"*/;

wire [CTRL_ADDR_WIDTH-1:0]  ddr_axi_araddr ;// 读地址
wire [3:0]                  ddr_axi_arlen  ;// 读Burst长度
wire                        ddr_axi_arready /*synthesis PAP_MARK_DEBUG="1"*/;// 读地址准备好
wire                        ddr_axi_arvalid  /*synthesis PAP_MARK_DEBUG="1"*/;// 读地址有效
wire [8*MEM_DQ_WIDTH-1:0]   ddr_axi_rdata  ;// 读数据（128bit）
wire                        ddr_axi_rvalid /*synthesis PAP_MARK_DEBUG="1"*/;// 读数据有效
wire                        ddr_axi_rlast/*synthesis PAP_MARK_DEBUG="1"*/;
// 跨时钟域FIFO信号（摄像头→DDR3）
wire                        fifo_wr_en     /*synthesis PAP_MARK_DEBUG="1"*/;  // FIFO写使能
wire                        fifo_full      /*synthesis PAP_MARK_DEBUG="1"*/;  // FIFO满
wire                        fifo_almost_full      /*synthesis PAP_MARK_DEBUG="1"*/;  // FIFO满
wire [FIFO_WIDTH-1:0]       fifo_wr_data   /*synthesis PAP_MARK_DEBUG="1"*/;  // FIFO写数据（32bit）
wire [10:0]                 fifo_wr_water_level;/*synthesis PAP_MARK_DEBUG="1"*/; 
wire                        fifo_rd_en     /*synthesis PAP_MARK_DEBUG="1"*/;  // FIFO读使能
wire                        fifo_empty     /*synthesis PAP_MARK_DEBUG="1"*/;  // FIFO空
wire                        fifo_almost_empty     /*synthesis PAP_MARK_DEBUG="1"*/;  // FIFO空
wire [FIFO_WIDTH-1:0]       fifo_rd_data   /*synthesis PAP_MARK_DEBUG="1"*/;  // FIFO读数据（32bit）
wire [10:0]                 fifo_rd_water_level;/*synthesis PAP_MARK_DEBUG="1"*/; 
// RGB565→RGB888转换信号
wire [23:0]                 cam_rgb888     /*synthesis PAP_MARK_DEBUG="1"*/;  // 转换后RGB888
//wire                        cam_rgb888_de  /*synthesis PAP_MARK_DEBUG="1"*/;  // RGB888数据有效

// 实例化RGB565→RGB888（摄像头1）
rgb565_to_rgb888 u_rgb565_to_rgb888(
    .rst_n          (cmos_init_done1  ),
    .i_rgb565       (cmos1_d_16bit    ),// 输入RGB565（来自cmos_8_16bit）
    .o_rgb888       (cam_rgb888       )// 输出RGB888
);

localparam DQ_WIDTH = 16;
 

    //写地址通道↓                                                  
    wire [3 : 0]                           M_AXI_AWID     /* synthesis PAP_MARK_DEBUG="1" */;
    wire [CTRL_ADDR_WIDTH-1 : 0]           M_AXI_AWADDR   /* synthesis PAP_MARK_DEBUG="1" */;
    //wire [3 : 0]                           M_AXI_AWLEN    /* synthesis PAP_MARK_DEBUG="1" */;
    wire                                   M_AXI_AWUSER   /* synthesis PAP_MARK_DEBUG="1" */;
    wire                                   M_AXI_AWVALID   /* synthesis PAP_MARK_DEBUG="1" */;
    wire                                   M_AXI_AWREADY   /* synthesis PAP_MARK_DEBUG="1" */;
    //写数据通道↓                                                 
    wire [DQ_WIDTH*8-1 : 0]                M_AXI_WDATA    /* synthesis PAP_MARK_DEBUG="1" */;
    wire [DQ_WIDTH-1 : 0]                  M_AXI_WSTRB    /* synthesis PAP_MARK_DEBUG="1" */;
    wire                                   M_AXI_WLAST    /* synthesis PAP_MARK_DEBUG="1" */;
    wire [3 : 0]                           M_AXI_WUSER    /* synthesis PAP_MARK_DEBUG="1" */;
    wire                                   M_AXI_WREADY   /* synthesis PAP_MARK_DEBUG="1" */;                                                
    //读地址通道↓                                                 
    wire [3 : 0]                           M_AXI_ARID     /* synthesis PAP_MARK_DEBUG="1" */;
    wire                                   M_AXI_ARUSER   /* synthesis PAP_MARK_DEBUG="1" */;
    wire [CTRL_ADDR_WIDTH-1 : 0]           M_AXI_ARADDR   /* synthesis PAP_MARK_DEBUG="1" */;
    //wire [3 : 0]                           M_AXI_ARLEN    /* synthesis PAP_MARK_DEBUG="1" */;
    wire                                   M_AXI_ARVALID   /* synthesis PAP_MARK_DEBUG="1" */;
    wire                                   M_AXI_ARREADY   /* synthesis PAP_MARK_DEBUG="1" */;
    //读数据通道↓                                                
    wire  [3 : 0]                          M_AXI_RID      /* synthesis PAP_MARK_DEBUG="1" */;
    wire  [DQ_WIDTH*8-1 : 0]               M_AXI_RDATA    /* synthesis PAP_MARK_DEBUG="1" */;
    wire                                   M_AXI_RLAST    /* synthesis PAP_MARK_DEBUG="1" */;
    wire                                   M_AXI_RVALID   /* synthesis PAP_MARK_DEBUG="1" */;



reg cmos1_vsync_d1/*synthesis PAP_MARK_DEBUG="1"*/;
reg cmos1_vsync_pos/*synthesis PAP_MARK_DEBUG="1"*/;
reg cmos1_vsync_neg/*synthesis PAP_MARK_DEBUG="1"*/;
// 检测vsync上升沿和下降沿（同步到cmos1_pclk_16bit域，适配低电平有效）
always @(posedge cmos1_pclk_16bit or negedge (global_rst_n && cmos_init_done1)) begin
    if (!(global_rst_n && cmos_init_done1)) begin
        cmos1_vsync_d1 <= 1'b0;
        cmos1_vsync_pos <= 1'b0;  // 改为：检测上升沿（对应帧结束）
        cmos1_vsync_neg <= 1'b0;  // 改为：检测下降沿（对应帧开始）
    end else begin
        cmos1_vsync_d1 <= cmos1_vsync_d0;  // 延迟1拍，用于边沿检测
        
        cmos1_vsync_neg <= !cmos1_vsync_d0 && cmos1_vsync_d1;  
       
        cmos1_vsync_pos <= cmos1_vsync_d0 && !cmos1_vsync_d1;  
    end
end
// 声明计数器（放在模块信号声明区）
reg [3:0] cmos1_vsync_pos_cnt;  // 4位计数器，可记录0~10
reg  cmos1_vsync_start_en = 1'b0/*synthesis PAP_MARK_DEBUG="1"*/;
// 记录cmos1_vsync_pos上升沿次数（最大10次）
always @(posedge cmos1_pclk_16bit or negedge (global_rst_n && cmos_init_done1)) begin
    if (!(global_rst_n && cmos_init_done1)) begin
        // 复位时计数器清零
        cmos1_vsync_pos_cnt <= 4'd0;
        cmos1_vsync_start_en <= 1'b0;
    end else begin
        // 当检测到上升沿（cmos1_vsync_pos为高）且未达到最大值时，计数+1
        if (cmos1_vsync_pos && cmos1_vsync_pos_cnt < 4'd10) begin
            cmos1_vsync_pos_cnt <= cmos1_vsync_pos_cnt + 4'd1;
        end else if(cmos1_vsync_pos && cmos1_vsync_pos_cnt == 4'd10 && !(myReg0_flag == 8'd0)) begin
            cmos1_vsync_start_en <= 1'b1;
        end
        // 达到10后保持不变（无需额外逻辑，不满足条件时自然保持）
    end
end

// 1. 第一步：在 ddr_core_clk 域统计 wr_flag 中0的个数（空闲区域数）
// （在原 wr_flag 控制逻辑的同一时钟域统计，确保计数准确）
reg [2:0] free_region_cnt_ddr; // 0~4，统计空闲区域数（0的个数）
always @(posedge ddr_core_clk or negedge global_rst_n) begin
    if (!global_rst_n) begin
        free_region_cnt_ddr <= 3'd4; // 复位时所有区域空闲（wr_flag=4'b0000）
    end else begin
        // 统计 wr_flag 中 0 的个数（空闲区域数）
        free_region_cnt_ddr <= (~wr_flag[0]) + (~wr_flag[1]) + (~wr_flag[2]) + (~wr_flag[3]);
        // 注：~wr_flag[i] 是1当且仅当 wr_flag[i]=0，求和即空闲区域数
    end
end

// 2. 第二步：将空闲区域数同步到 cmos1_pclk_16bit 域（跨时钟域同步，避免亚稳态）
reg [2:0] free_region_cnt_sync1, free_region_cnt_sync2;
always @(posedge cmos1_pclk_16bit or negedge (global_rst_n && cmos_init_done1)) begin
    if (!(global_rst_n && cmos_init_done1)) begin
        free_region_cnt_sync1 <= 3'd0;
        free_region_cnt_sync2 <= 3'd0;
    end else begin
        // 2级同步：跨时钟域信号必须同步，消除亚稳态
        free_region_cnt_sync1 <= free_region_cnt_ddr;
        free_region_cnt_sync2 <= free_region_cnt_sync1;
    end
end

// 3. 第三步：实现 cmos1_vsync_empty_en（允许接收新帧的使能）
reg cmos1_vsync_empty_en;
always @(posedge cmos1_pclk_16bit or negedge (global_rst_n && cmos_init_done1)) begin
    if (!(global_rst_n && cmos_init_done1)) begin
        
        cmos1_vsync_empty_en <= 1'b0;
    end else begin
       
        if (cmos1_vsync_pos) begin
           
            cmos1_vsync_empty_en <= (free_region_cnt_sync2 >= 3'd2) ? 1'b1 : 1'b0;
        end else begin
            
            cmos1_vsync_empty_en <= cmos1_vsync_empty_en;
        end
    end
end
// 生成fifo_fill_byte（包含正确帧标志，适配低电平有效） 第一个像素是FE开头
always @(posedge cmos1_pclk_16bit or negedge (global_rst_n && cmos_init_done1)) begin
    if (!(global_rst_n && cmos_init_done1)) begin
        fifo_fill_byte <= 8'd0;
    end else begin
        if (cmos1_vsync_neg) begin  
            fifo_fill_byte <= 8'hFE; 
        end else if (buf_en) begin  // 正常数据，填充字节递增
                fifo_fill_byte <= (fifo_fill_byte == 8'hFC || fifo_fill_byte == 8'hFE) ? 8'd0 : fifo_fill_byte + 8'd1;
        end else begin
            fifo_fill_byte <= fifo_fill_byte;
        end        
    end
end



// ========================== 新增：0~255递增填充变量（cam_rgb888_clk时钟域） ==========================
// 定义8位递增寄存器（填充到FIFO写数据的高8位）
reg [7:0] fifo_fill_byte = 8'd0; 

reg [23:0] counter_24bit;
always @(posedge cmos1_pclk_16bit or negedge (global_rst_n && cmos_init_done1)) begin
    // 异步复位：复位有效时（global_rst_n无效或cmos_init_done1无效），计数器清零
    if (!(global_rst_n && cmos_init_done1)) begin
        counter_24bit <= 24'd0;  // 初始化为0
    end else begin
        // 仅当FIFO写使能有效且非满时，计数器递增
        if  (buf_en)  begin
            // 达到最大值0xFFFFFF（24位全1）时，循环复位为0
            if (counter_24bit == 24'hFFFFFF) begin
                counter_24bit <= 24'd0;
            end else begin
                counter_24bit <= counter_24bit + 24'd1;  // 未达最大值时，每次+1
            end
        end else begin
            // 写使能无效或FIFO已满时，保持当前值
            counter_24bit <= counter_24bit;
        end
    end
end




reg [127:0] data_buf_128;  // 分4段：[31:0]第1周期、[63:32]第2周期、[95:64]第3周期、[127:96]第4周期
reg [2:0]   buf_cnt;       // 缓存计数器（0~7）：记录当前缓存了几个周期的数据
wire        buf_full/*synthesis PAP_MARK_DEBUG="1"*/;        // 缓存满标志（buf_cnt==3，即4个周期数据已存满）
wire        buf_en;        // 缓存使能：数据有效时才缓存（避免无效数据）

// 缓存满标志：计数器到3表示4个数据存满
assign buf_full = (buf_cnt == 3'd4);

assign buf_en = cmos1_href_16bit && cmos_init_done1 && cmos1_vsync_start_en && cmos1_vsync_empty_en;
// 4周期数据缓存逻辑：每周期存1个32位数据，满4个后拼接成128位
always @(posedge cmos1_pclk_16bit or negedge (global_rst_n && cmos_init_done1)) begin
    if (!(global_rst_n && cmos_init_done1)) begin
        data_buf_128 <= 128'd0;  // 复位：缓存清零
        buf_cnt <= 3'd0;         // 复位：计数器清零
    end else begin
          if (buf_en) begin  // 数据有效时，才进行缓存
            // 1. 按计数器值，将当前周期的32位数据存入缓存对应段
            case(buf_cnt)

                3'd0: data_buf_128[31:0]   <= {fifo_fill_byte, cam_rgb888};  // 第1周期：低32位
                3'd1: data_buf_128[63:32]  <= {fifo_fill_byte, cam_rgb888};  // 第2周期：中低32位
                3'd2: data_buf_128[95:64]  <= {fifo_fill_byte, cam_rgb888};  // 第3周期：中高32位
                3'd3: data_buf_128[127:96] <= {fifo_fill_byte, cam_rgb888};  // 第4周期：高32位
                3'd4: data_buf_128[31:0]   <= {fifo_fill_byte, cam_rgb888};  // 第1周期：低32位

            endcase
            // 2. 计数器递增（满3后自动清零，避免溢出）
            if(buf_cnt == 3'd4) begin
                buf_cnt <=  3'd1;
            end else begin
                buf_cnt <= buf_cnt +  3'd1;
            end
        end else if(buf_cnt == 3'd4) begin
                buf_cnt <=  3'd0;
        end
    end
end
// FIFO写使能：缓存满（4个数据凑齐）+ FIFO未满 + 数据有效
assign fifo_wr_en =( buf_full || cmos1_vsync_pos) && !fifo_full && cmos1_vsync_start_en && cmos1_vsync_empty_en;
//assign fifo_wr_en = buf_full && !fifo_full && cmos1_vsync_start_en ;
// FIFO写数据：缓存满时，将128位拼接数据写入FIFO（否则写0，不影响）
assign fifo_wr_data = !cmos1_vsync_pos ? data_buf_128 : VSYNC_POS_MARK;

// 写复位同步到cmos1_pclk_16bit
reg [1:0] wr_rst_sync;
always @(posedge cmos1_pclk_16bit or negedge global_rst_n) begin
    if (!global_rst_n) wr_rst_sync <= 2'b11;
    else wr_rst_sync <= {wr_rst_sync[0], ~cmos_init_done1};
end
assign fifo_wr_rst = wr_rst_sync[1];

SXT1_FIFO u_SXT1_FIFO_to_DDR (
  .wr_clk(cmos1_pclk_16bit),                    // input
  .wr_rst(fifo_wr_rst),                    // input
  .wr_en(fifo_wr_en),                      // input
  .wr_data(fifo_wr_data),                  // input [127:0]
  .wr_full(fifo_full),                  // output
  .wr_water_level(fifo_wr_water_level),    // output [10:0]
  .almost_full(fifo_almost_full),          // output

  .rd_clk(ddr_core_clk),                    // input
  .rd_rst(~(global_rst_n && ddr_init_done)),                    // input
  .rd_en(fifo_rd_en),                      // input
  .rd_data(fifo_rd_data),                  // output [127:0]
  .rd_empty(fifo_empty),                // output
  .rd_water_level(fifo_rd_water_level),    // output [10:0]
  .almost_empty(fifo_almost_empty)         // output
);

assign fifo_rd_en =   (ddr_axi_wready || fifo_rd_en_early) && ddr_init_done;
reg fifo_rd_en_early;
reg fifo_rd_en_early_reg;

always @(posedge ddr_core_clk or negedge global_rst_n) begin
    if (!global_rst_n) begin
        fifo_rd_en_early <= 1'b0;   
        fifo_rd_en_early_reg <= 1'b1; 
    end else begin        
        if (fifo_rd_en_early_reg == 1'b1 && !fifo_almost_empty) begin
              fifo_rd_en_early <= 1'b1;
              fifo_rd_en_early_reg <= 1'b0; 
        end else begin
              fifo_rd_en_early <= 1'b0;
        end
    end
end
//assign fifo_rd_en = !fifo_almost_empty && ddr_axi_awvalid_en_neg && ddr_init_done;
// ========================== 6. 实例化DDR3 IP（来自DDR3_IP_源码.docx） ==========================
wire   pll_lock_ip/*synthesis PAP_MARK_DEBUG="1"*/;  
wire   phy_pll_lock_ip/*synthesis PAP_MARK_DEBUG="1"*/;  
wire   ddrphy_cpd_lock_ip/*synthesis PAP_MARK_DEBUG="1"*/;  
wire   dbg_ddrphy_init_fail_ip/*synthesis PAP_MARK_DEBUG="1"*/;  
// 紫光FPGA全局复位释放模块（必需！无此模块DDR3复位无法解除）
GTP_GRS GTP_GRS_INST(
    .GRS_N(1'b1)  // 高电平有效，释放全局复位（参考例程默认配置）
);
ddr3 #(
    .MEM_ROW_WIDTH       (MEM_ROW_WIDTH    ),
    .MEM_COLUMN_WIDTH    (MEM_COLUMN_WIDTH ),
    .MEM_BANK_WIDTH      (MEM_BANK_WIDTH   ),
    .MEM_DQ_WIDTH        (MEM_DQ_WIDTH     ),
    .MEM_DM_WIDTH        (MEM_DM_WIDTH     ),
    .MEM_DQS_WIDTH       (MEM_DQS_WIDTH    ),
    .CTRL_ADDR_WIDTH     (CTRL_ADDR_WIDTH  )
) u_ddr3 (
    // 时钟复位
    .ref_clk              (sys_clk         ),// 输入参考时钟
    .resetn               (rstn_out            ),// 输入全局复位
    .core_clk             (ddr_core_clk     ),// 输出DDR3核心时钟
    .pll_lock              (pll_lock_ip),                // 输出PLL锁定（未使用）
    .phy_pll_lock          (phy_pll_lock_ip),                // 输出PHY PLL锁定（未使用）
    .gpll_lock             (),                // 输出GPL锁定（未使用）
    .rst_gpll_lock         (),                // 输出GPL复位锁定（未使用）
    .ddrphy_cpd_lock       (ddrphy_cpd_lock_ip),                // 输出PHY CPD锁定（未使用）
    .ddr_init_done         (ddr_init_done   ),// 输出DDR3初始化完成
    // AXI写接口（摄像头→DDR3）
    .axi_awaddr           (ddr_axi_awaddr   ),// 输入写地址
    .axi_awuser_ap        (1'b0             ),// 输入AP标志（未使用）
    .axi_awuser_id        (4'd0             ),// 输入写ID（0）
    .axi_awlen            (ddr_axi_awlen    ),// 输入Burst长度
    .axi_awready          (ddr_axi_awready  ),// 输出地址准备好
    .axi_awvalid          (ddr_axi_awvalid  ),// 输入地址有效
    .axi_wdata            (ddr_axi_wdata    ),// 输入写数据
    .axi_wstrb            (ddr_axi_wstrb    ),// 输入字节使能
    .axi_wready           (ddr_axi_wready   ),// 输出数据准备好
    .axi_wusero_id        (),                 // 输出响应ID（未使用）
    .axi_wusero_last      (ddr_axi_wusero_last),                 // 输出最后一拍（未使用）
    // AXI读接口（DDR3→BAR0）
    .axi_araddr           (ddr_axi_araddr   ),// 输入读地址
    .axi_aruser_ap        (1'b0             ),// 输入AP标志（未使用）
    .axi_aruser_id        (4'd1             ),// 输入读ID（1）
    .axi_arlen            (ddr_axi_arlen    ),// 输入Burst长度
    .axi_arready          (ddr_axi_arready  ),// 输出读地址准备好
    .axi_arvalid          (ddr_axi_arvalid  ),// 输入读地址有效
    .axi_rdata            (ddr_axi_rdata    ),// 输出读数据（128bit）
    .axi_rid              (),                 // 输出读ID（未使用）
    .axi_rlast            (ddr_axi_rlast),                 // 输出读最后一拍（未使用）
    .axi_rvalid           (ddr_axi_rvalid   ),// 输出读数据有效
    // APB配置（未使用）
    .apb_clk              (1'b0             ),
    .apb_rst_n            (1'b1             ),
    .apb_sel              (1'b0             ),
    .apb_enable           (1'b0             ),
    .apb_addr             (8'b0             ),
    .apb_write            (1'b0             ),
    .apb_ready            (),
    .apb_wdata            (16'b0            ),
    .apb_rdata            (),
    // DDR3硬件引脚（按RK3568硬件手册连接）
    .mem_cs_n             (ddr_mem_cs_n     ),
    .mem_rst_n            (ddr_mem_rst_n    ),
    .mem_ck               (ddr_mem_ck       ),
    .mem_ck_n             (ddr_mem_ck_n     ),
    .mem_cke              (ddr_mem_cke      ),
    .mem_ras_n            (ddr_mem_ras_n    ),
    .mem_cas_n            (ddr_mem_cas_n    ),
    .mem_we_n             (ddr_mem_we_n     ),
    .mem_odt              (ddr_mem_odt      ),
    .mem_a                (ddr_mem_a        ),
    .mem_ba               (ddr_mem_ba       ),
    .mem_dqs              (ddr_mem_dqs      ),
    .mem_dqs_n            (ddr_mem_dqs_n    ),
    .mem_dq               (ddr_mem_dq       ),
    .mem_dm               (ddr_mem_dm       ),
    // 调试信号（未使用） .dbg_ddrphy_rst_n     (1'b1             ),
    .dbg_gate_start       (1'b0             ),
    .dbg_cpd_start        (1'b0             ),
    .dbg_ddrphy_rst_n     (1'b1             ),
    .dbg_gpll_scan_rst    (1'b0             ),
    .samp_position_dyn_adj (1'b0            ),
    .init_samp_position_even (16'b0         ),
    .init_samp_position_odd (16'b0          ),
    .wrcal_position_dyn_adj (1'b0           ),
    .init_wrcal_position  (16'b0            ),
    .force_read_clk_ctrl  (1'b0             ),
    .init_slip_step       (16'b0            ),
    .init_read_clk_ctrl   (12'b0            ),
    .debug_calib_ctrl     (),
    .dbg_slice_status     (),
    .dbg_slice_state      (),
    .debug_data           (),
    .dbg_dll_upd_state    (),
    .debug_gpll_dps_phase (),
    .dbg_rst_dps_state    (),
    .dbg_tran_err_rst_cnt (),
    .dbg_ddrphy_init_fail (dbg_ddrphy_init_fail_ip),
    .debug_cpd_offset_adj (1'b0             ),
    .debug_cpd_offset_dir (1'b0             ),
    .debug_cpd_offset     (10'b0            ),
    .debug_dps_cnt_dir0   (),
    .debug_dps_cnt_dir1   (),
    .ck_dly_en            (1'b0             ),
    .init_ck_dly_step     (8'b0             ),
    .ck_dly_set_bin       (),
    .align_error          (),
    .debug_rst_state      (),
    .debug_cpd_state      ()
);

// ========================== 7. DDR3读写逻辑（适配RGB888） ==========================
// 7.1 DDR3写配置（RGB888：24bit→16bit×2传输）
// 调整ddr_axi_awlen：正常传输用7（8个数据），特殊传输用0（1个数据）
assign ddr_axi_awlen = is_special_trans ? 4'd0 : 4'd7;  // 关键修改          // Burst长度
assign ddr_axi_wstrb = 16'b1111_1111_1111_1111;// 字节使能
assign ddr_axi_wdata = fifo_rd_data;    // 写数据=FIFO读数据（32bit）
// 地址拼接：高二位（区域）+ 低26位（计数）
assign ddr_axi_awaddr = {curr_wr_reg, ddr_wr_cnt_26};
assign ddr_axi_awvalid = (fifo_rd_water_level > 11'd8) && ddr_init_done && (!counter_24bit_end) && ddr_axi_awvalid_en && !ddr_axi_awvalid_en_end && (wr_flag[curr_wr_reg] == 1'b0);  // 只要有数据且IP就绪，就发起写地址请求，观察规律
// 写地址计数：
//reg [1:0]  region_sel_wr;          // 高二位：区域选择（00~11对应4个区域）
reg [25:0] ddr_wr_cnt_26;       // 低26位：区域内递增计数
reg counter_24bit_end = 1'b0;
reg ddr_axi_awvalid_en/*synthesis PAP_MARK_DEBUG="1"*/; 
// 声明延迟寄存器和下降沿标志（放在模块信号声明区）
reg ddr_axi_awvalid_en_d1;      // 存储上一拍的ddr_axi_awvalid_en
reg ddr_axi_awvalid_en_neg/*synthesis PAP_MARK_DEBUG="1"*/;     // 下降沿标志（单周期高脉冲）
reg ddr_axi_awvalid_en_end/*synthesis PAP_MARK_DEBUG="1"*/;
localparam DDR_NORMAL_STEP = 26'd64;  // 正常：64地址单位=128字节（BL8）
localparam DDR_SPECIAL_STEP = 26'd8;  // 特殊：8地址单位=16字节（BL1）
always @(posedge ddr_core_clk or negedge global_rst_n) begin
    if (!global_rst_n) begin
        ddr_wr_cnt_26 <= 26'd0;    // 低26位计数器复位
        ddr_axi_awvalid_en <= 1'b1;
        ddr_axi_awvalid_en_end <= 1'b0;
    end else begin
         if (ddr_axi_awvalid_en_end == 1'b1 || counter_24bit_end) begin
            //region_sel_wr <= region_sel_wr +1'd1;
            ddr_wr_cnt_26 <= 26'd0;
            counter_24bit_end <= 1'b0;
            ddr_axi_awvalid_en_end <= 1'b0;
        //end  else if (ddr_axi_awvalid && ddr_axi_awready && cmos1_vsync_neg_1 && wr_flag[curr_wr_reg] == 1'b0 ) begin
        end  else if (ddr_axi_awvalid && ddr_axi_awready ) begin
            if (ddr_wr_cnt_26 >= 26'd67108863 - 26'd64) begin // 预留1次地址增量
                counter_24bit_end <= 1'b1;
            end else if (is_special_trans)begin
                ddr_wr_cnt_26 <= ddr_wr_cnt_26 +  DDR_SPECIAL_STEP;
                ddr_axi_awvalid_en_end <= 1'b1;
            end else begin
                ddr_wr_cnt_26 <= ddr_wr_cnt_26 + DDR_NORMAL_STEP;
            end
                ddr_axi_awvalid_en <=1'b0;
        end 
        if(ddr_axi_wusero_last ) begin
            ddr_axi_awvalid_en <=1'b1;
        end
    end  
end
// 3. 传输次数计数与特殊传输标记（保持与之前一致，确保is_special_trans正确）
reg [14:0] ddr_trans_cnt/*synthesis PAP_MARK_DEBUG="1"*/;
wire is_special_trans/*synthesis PAP_MARK_DEBUG="1"*/;
always @(posedge ddr_core_clk or negedge global_rst_n) begin
    if (!global_rst_n) begin
        ddr_trans_cnt <= 15'd0;   
    end else begin        
        // 每次DDR写握手成功（1次传输完成），计数递增
        if (ddr_axi_awvalid && ddr_axi_awready) begin
            ddr_trans_cnt <= (ddr_trans_cnt == 15'd28800) ? 15'd0 : ddr_trans_cnt + 15'd1;
        end
    end
end
assign is_special_trans = (ddr_trans_cnt == 15'd28800);
// 声明延迟寄存器和上升沿标志信号（放在模块信号声明区）
reg ddr_axi_wready_d1;          // 存储上一拍的ddr_axi_wready
reg ddr_axi_wready_pos;         // 上升沿标志（单周期高脉冲）
reg ddr_axi_rvalid_d1;          // 新增：存储上一拍的ddr_axi_rvalid（用于上升沿对比）
reg ddr_axi_rvalid_pos;         // 新增：ddr_axi_rvalid上升沿标志（单周期高脉冲）
reg ddr_axi_rvalid_pos_reg/*synthesis PAP_MARK_DEBUG="1"*/; 


reg    eth0_rx_de;
reg    eth0_rx_vs;
reg   [15:0]  eth0_rx_data;
// 1. 延迟寄存器更新：同步到ddr_core_clk时钟域
always @(posedge ddr_core_clk or negedge global_rst_n) begin
    if (!global_rst_n) begin
        // 复位时清零，避免复位后误触发
        ddr_axi_wready_d1 <= 1'b0;
        //ddr_axi_rvalid_d1 <= 1'b0;
        // 复位时清零，避免初始状态误触发
        ddr_axi_awvalid_en_d1 <= 1'b0;
    end else begin
        // 每拍将当前ddr_axi_wready存入延迟寄存器
        ddr_axi_wready_d1 <= ddr_axi_wready;
        //ddr_axi_rvalid_d1 <= ddr_axi_rvalid;
        // 每拍将当前ddr_axi_awvalid_en存入延迟寄存器
        ddr_axi_awvalid_en_d1 <= ddr_axi_awvalid_en;
    end
end

// 2. 上升沿检测：当前高电平且上一拍低电平
always @(posedge ddr_core_clk or negedge global_rst_n) begin
    if (!global_rst_n) begin
        // 复位时标志清零
        ddr_axi_wready_pos <= 1'b0;
        //ddr_axi_rvalid_pos <= 1'b0;
        // 复位时标志清零
        ddr_axi_awvalid_en_neg <= 1'b0;
    end else begin
        // 上升沿判定条件：当前信号为高，且上一拍为低
        ddr_axi_wready_pos <= ddr_axi_wready && !ddr_axi_wready_d1;
        //ddr_axi_rvalid_pos <= ddr_axi_rvalid && !ddr_axi_rvalid_d1;
        // 下降沿判定条件：当前信号为低，且上一拍为高
        ddr_axi_awvalid_en_neg <= !ddr_axi_awvalid_en && ddr_axi_awvalid_en_d1;
    end
end
reg cmos1_vsync_neg_1;//摄像头1图片开始
//reg cmos1_vsync_pos_1;//摄像头1图片结束
// 定义检测条件：4个字节段中任意一个为8'hFE
wire has_ff_segment = (fifo_rd_data[31:24]  == 8'hFE) ||  // 第31-24位（第4个字节）
                      (fifo_rd_data[63:56]  == 8'hFE) ||  // 第63-56位（第8个字节）
                      (fifo_rd_data[95:88]  == 8'hFE) ||  // 第95-88位（第12个字节）
                      (fifo_rd_data[127:120] == 8'hFE);   // 第127-120位（第16个字节）
// 定义同步标记（与写入时的标记一致）
localparam VSYNC_POS_MARK = 128'hFD00_0000_FD00_0000_FD00_0000_FD00_0000;
// 1. 定义当前周期是否匹配标记
wire current_match = (fifo_rd_data == VSYNC_POS_MARK);
// 2. 寄存上一周期的匹配状态（用于检测上升沿）
reg prev_match;

always @(posedge ddr_core_clk or negedge global_rst_n) begin
    if (!global_rst_n) begin
        prev_match <= 1'b0;  // 复位时上一周期状态为0
    end else begin
        prev_match <= current_match;  // 每个周期更新为当前状态
    end
end
// 检测上升沿：当前匹配且上一周期不匹配
wire pos_edge_det ;
assign pos_edge_det = !current_match && prev_match;
always @(posedge ddr_core_clk or negedge global_rst_n) begin
    if (!global_rst_n) begin
        cmos1_vsync_neg_1 <= 1'b0;  // 复位清零
       // cmos1_vsync_pos_1 <= 1'b0;  // 复位清零
    end else begin      
        // 条件1：检测到任意FF段，置位cmos1_vsync_neg_1
        if (has_ff_segment) begin 
            cmos1_vsync_neg_1 <= 1'b1; 
        end 
        else if (pos_edge_det) begin
            cmos1_vsync_neg_1 <= 1'b0;
        end 
        // 其他情况：保持当前值（或根据需求清零，此处默认保持）
        else begin
            cmos1_vsync_neg_1 <= cmos1_vsync_neg_1;
        end
    end 
end
// 区域编号定义（0-3对应区域1-4）
localparam REGION_1 = 2'd0;
localparam REGION_2 = 2'd1;
localparam REGION_3 = 2'd2;
localparam REGION_4 = 2'd3;

// 读有效信号：当前读区域已写完（标记为1）
//assign rd_valid = wr_flag[curr_rd_reg];

reg [1:0] curr_wr_reg ;
reg [1:0] curr_rd_reg ;
reg [3:0] wr_flag;
// 合并写区域和读区域控制逻辑（同一时钟/复位域）
always @(posedge ddr_core_clk or negedge global_rst_n) begin
    if (!global_rst_n) begin
        // 复位初始化
        curr_wr_reg <= REGION_1;  // 初始写区域：区域1
        curr_rd_reg <= REGION_1;  // 初始读区域：区域1
        wr_flag     <= 4'b0000;   // 初始无区域写完标记
    end else begin
        // 1. 写区域逻辑：检测到区域写完（pos_edge_det有效）时更新标记并切换区域
        if (wr_flag[curr_wr_reg] == 1'b0 && (ddr_axi_awvalid_en_end == 1'b1  || counter_24bit_end)) begin
            wr_flag[curr_wr_reg] <= 1'b1;  // 标记当前写区域为“已写完”
            
            // 循环切换写区域（1→2→3→4→1）
            case (curr_wr_reg)
                REGION_1: curr_wr_reg <= REGION_2;
                REGION_2: curr_wr_reg <= REGION_3;
                REGION_3: curr_wr_reg <= REGION_4;
                REGION_4: curr_wr_reg <= REGION_1;
                default:  curr_wr_reg <= REGION_1;
            endcase
        end

        // 2. 读区域逻辑：当前区域可读且读使能有效时，清除标记并切换区域
        // （注：若与写逻辑在同一周期触发，读逻辑后执行，不影响写标记的更新）
        if (wr_flag[curr_rd_reg] == 1'b1 && sync_cmos1_vsync_pos_1_neg) begin
            wr_flag[curr_rd_reg] <= 1'b0;  // 清除当前读区域的“已写完”标记
            
            // 循环切换读区域（1→2→3→4→1）
            case (curr_rd_reg)
                REGION_1: curr_rd_reg <= REGION_2;
                REGION_2: curr_rd_reg <= REGION_3;
                REGION_3: curr_rd_reg <= REGION_4;
                REGION_4: curr_rd_reg <= REGION_1;
                default:  curr_rd_reg <= REGION_1;
            endcase
        end
    end
end
// 7.2 DDR3读配置（适配BAR0 128bit）
assign ddr_axi_arlen = 4'd7;            // Burst长度
//assign ddr_axi_arvalid =  (blk_state == WRITE) && ddr_init_done && (ddr_rd_cnt+8 <= ddr_wr_cnt) && !ddr_fifo_full; // 新增：FIFO未满才发起DDR3读请求
//assign ddr_axi_arvalid =  (blk_state == WRITE) && ddr_init_done&& !ddr_almost_full &&(wr_flag[curr_rd_reg] == 1'b1) && ddr_axi_arvalid_en; // 新增：FIFO未满才发起DDR3读请求  观察规律
assign ddr_axi_arvalid =  (blk_state == WRITE) && ddr_init_done&& (ddr_fifo_wr_water_level < 11'd1016) &&(wr_flag[curr_rd_reg] == 1'b1)  && ddr_axi_arvalid_en; // 
assign ddr_axi_araddr = {curr_rd_reg, ddr_rd_cnt_26};   // 读地址=计数器值
// 读地址计数：
reg [25:0] ddr_rd_cnt_26 = 26'd0;       // 低26位：区域内递增计数
reg ddr_axi_arvalid_en/*synthesis PAP_MARK_DEBUG="1"*/; 
// 读地址计数：
always @(posedge ddr_core_clk or negedge global_rst_n) begin
    if (!global_rst_n) begin
        ddr_rd_cnt_26 <= 26'd0;
        ddr_axi_arvalid_en <=1'b1;
        //ddr_axi_rvalid_pos_reg <= 1'b0;
    end else begin
        if (ddr_axi_arvalid && ddr_axi_arready) begin
            //if (ddr_rd_cnt_26 >= 28'd268435455 - 28'd8) begin
            if ((ddr_rd_cnt_26 >=( 26'd67108863 -  26'd64)) || sync_cmos1_vsync_pos_1_neg) begin // 预留1次地址增量
            //  if (ddr_rd_cnt >= 28'd32768-28'd8) begin   //观察规律 32768*2=65536
                ddr_rd_cnt_26 <= 26'd0;
            end else begin
                ddr_rd_cnt_26 <= ddr_rd_cnt_26 +  26'd64;
            end
                ddr_axi_arvalid_en <=1'b0;
        end else if(sync_cmos1_vsync_pos_1_neg) begin
                ddr_rd_cnt_26 <= 26'd0;
        end
        if(ddr_axi_rlast) begin
            ddr_axi_arvalid_en <=1'b1;
        end
    end
end

/*eth0_img_rec u_eth0_img_rec (
    .eth_rx_clk   ( clk100m      ),
    .img_data     ( eth0_rx_data   )
); */
/*
       if( ddr_axi_rvalid_pos_reg && !ddr_almost_full) begin
            ddr_axi_arvalid_en <=1'b1;
            ddr_axi_rvalid_pos_reg <=1'b0;
        end else if(ddr_axi_rvalid_pos) begin
            ddr_axi_rvalid_pos_reg <=1'b1;
        end
*/
///////////////////////////////////摄像头////////////////////////////////////////
// ========================== DDR3→状态机 跨时钟域FIFO ==========================

localparam DDR_BAR0_FIFO_WIDTH = 128;  // 数据位宽=DDR3读数据位宽（128bit）
localparam DDR_BAR0_FIFO_DEPTH = 10'd512;// 深度512，适配突发数据（可根据需求调整）

// FIFO信号声明
wire                        ddr_fifo_wr_en/*synthesis PAP_MARK_DEBUG="1"*/;    // FIFO写使能（ddr_core_clk域）
wire                        ddr_fifo_full/*synthesis PAP_MARK_DEBUG="1"*/;     // FIFO满（ddr_core_clk域）
wire                        ddr_almost_full/*synthesis PAP_MARK_DEBUG="1"*/; 
wire [DDR_BAR0_FIFO_WIDTH-1:0] ddr_fifo_wr_data/*synthesis PAP_MARK_DEBUG="1"*/;   // FIFO写数据（ddr_axi_rdata）
wire       [10:0]           ddr_fifo_wr_water_level;

wire                        ddr_fifo_rd_en/*synthesis PAP_MARK_DEBUG="1"*/;    // FIFO读使能（clk_100m域）
wire                        ddr_fifo_empty/*synthesis PAP_MARK_DEBUG="1"*/;    // FIFO空（clk_100m域）
wire                        ddr_almost_empty/*synthesis PAP_MARK_DEBUG="1"*/; 
wire [DDR_BAR0_FIFO_WIDTH-1:0] ddr_fifo_rd_data/*synthesis PAP_MARK_DEBUG="1"*/;   // FIFO读数据（同步到clk_100m域）

// ========================== DDR3→状态机 跨时钟域FIFO ==========================
// 读复位同步到clk_100m
reg [1:0] rd_rst_sync;
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) rd_rst_sync <= 2'b11;
    else rd_rst_sync <= {rd_rst_sync[0], ~ddr_init_done};
end
assign fifo_rd_rst = rd_rst_sync[1];

SXT1_FIFO u_SXT1_FIFO_to_PCIe (
  .wr_clk(ddr_core_clk),                    // input
  .wr_rst(~(global_rst_n && ddr_init_done)),                    // input
  .wr_en(ddr_fifo_wr_en),                      // input
  .wr_data(ddr_fifo_wr_data),                  // input [127:0]
  .wr_full(ddr_fifo_full),                  // output
  .wr_water_level(ddr_fifo_wr_water_level),    // output [10:0]
  .almost_full(ddr_almost_full),          // output

  .rd_clk(clk_100m),                    // input
  .rd_rst(~rst_n),                    // input
  .rd_en(ddr_fifo_rd_en),                      // input
  .rd_data(ddr_fifo_rd_data),                  // output [127:0]
  .rd_empty(ddr_fifo_empty),                // output
  .rd_water_level(),    // output [10:0]
  .almost_empty(ddr_almost_empty)         // output
);
// FIFO写端配置（ddr_core_clk域）
assign ddr_fifo_wr_data = ddr_axi_rdata;  // 写数据=DDR3读数据
assign ddr_fifo_wr_en = ddr_axi_rvalid;    // 写使能=DDR3读数据有效（数据就绪）.
assign ddr_fifo_rd_en = ddr_fifo_rd_en_pcie || ddr_fifo_rd_en_early;
///////////////////////////////////IIC//////////////////////////////////////////
//IIC register                                   
wire  [7:0]myReg0_flag;
wire  [7:0] myReg1_flag;

wire    clk_100m    ;
wire    rst_n       ;
reg   Reg0_wr_en;
reg   Reg1_wr_en;
reg   [7:0]  myReg0_w;
reg   [7:0]  myReg1_w;
reg    Reg_wr_state;

//iic slave模块
i2cSlave i2cSlave_u (
	.clk		(clk_100m),		
	//.rst		(~locked  ),		
    .rst		(~rst_n  ),		
	.sda		(i2c_sda ),		
	.scl		(i2c_scl ),	
    .Reg0_wr_en (Reg0_wr_en ),		
    .Reg1_wr_en (Reg1_wr_en ),		
    .myReg0_w   (myReg0_w ),		
	.myReg1_w   (myReg1_w ),		
	.myReg0		(myReg0_flag ),		
	.myReg1		(myReg1_flag )		
);

///////////////////////////////////IIC//////////////////////////////////////////
localparam DEVICE_TYPE = 3'b000;			// @IPC enum 3'b000, 3'b001, 3'b100（EP模式，不修改）
localparam AXIS_SLAVE_NUM = 3;				// @IPC enum 1 2 3（AXI从机数量，不修改）

// Test unit mode signals（原有信号，不修改）
wire			pcie_cfg_ctrl_en;			
wire			axis_master_tready_cfg;		

wire			cfg_axis_slave0_tvalid;		
wire	[127:0]	cfg_axis_slave0_tdata;		
wire			cfg_axis_slave0_tlast;		
wire			cfg_axis_slave0_tuser;		

// For mux（原有信号，不修改）
wire			axis_master_tready_mem;		
wire			axis_master_tvalid_mem;		
wire	[127:0]	axis_master_tdata_mem;		
wire	[3:0]	axis_master_tkeep_mem;			
wire			axis_master_tlast_mem;		
wire	[7:0]	axis_master_tuser_mem;		

wire			cross_4kb_boundary;			

wire			dma_axis_slave0_tvalid;		
wire	[127:0]	dma_axis_slave0_tdata;		
wire			dma_axis_slave0_tlast;		
wire			dma_axis_slave0_tuser;		

// Reset debounce and sync（原有信号， ========== 修改处 ========== 补充sync_perst_n声明）
wire			sync_button_rst_n; 			
wire			sync_perst_n;					// 新增：显式声明PCIe复位防抖后信号（原代码隐式声明，避免编译警告）
wire			ref_core_rst_n;				
wire			s_pclk_rstn;				

// Internal signal（原有信号，不修改）
wire			pclk_div2/*synthesis PAP_MARK_DEBUG="1"*/;  	// 用户时钟，x2 5gt/s时125MHZ，2.5gt/s时62.5MHZ			
wire			pclk/*synthesis PAP_MARK_DEBUG="1"*/;			// 用户时钟，同pclk_div2（此处代码中两者一致，按实际IP输出）			
wire			ref_clk; 					
wire			core_rst_n;					

wire			axis_master_tvalid;
wire			axis_master_tready;
wire	[127:0]	axis_master_tdata;
wire	[3:0]	axis_master_tkeep;
wire			axis_master_tlast;
wire	[7:0]	axis_master_tuser;
wire             dma_axis_master_tready;

// AXI4-Stream slave 0 interface
wire			axis_slave0_tready;
wire			axis_slave0_tvalid;
wire	[127:0]	axis_slave0_tdata;
wire			axis_slave0_tlast;
wire			axis_slave0_tuser;
// AXI4-Stream slave 1 interface（未使用，保持原有）
wire			axis_slave1_tready;
wire			axis_slave1_tvalid;
wire	[127:0]	axis_slave1_tdata;
wire			axis_slave1_tlast;
wire			axis_slave1_tuser;
// AXI4-Stream slave 2 interface（未使用，保持原有）
wire			axis_slave2_tready;
wire			axis_slave2_tvalid;
wire	[127:0]	axis_slave2_tdata;
wire			axis_slave2_tlast;
wire			axis_slave2_tuser;

// 原有信号（不修改）
wire	[7:0]	cfg_pbus_num;			
wire	[4:0]	cfg_pbus_dev_num; 		
wire	[2:0]	cfg_max_rd_req_size;	
wire	[2:0]	cfg_max_payload_size;	
wire			cfg_rcb;				

wire			cfg_ido_req_en;			
wire			cfg_ido_cpl_en;			
wire	[7:0]	xadm_ph_cdts;			
wire	[11:0]	xadm_pd_cdts;			
wire	[7:0]	xadm_nph_cdts;			
wire	[11:0]	xadm_npd_cdts;			
wire	[7:0]	xadm_cplh_cdts;			
wire	[11:0]	xadm_cpld_cdts;			

wire	[4:0]	smlh_ltssm_state/*synthesis PAP_MARK_DEBUG="1"*/;//link状态机（调试用）

// Led lights up signal（原有信号，不修改）
reg		[22:0]	ref_led_cnt;		
reg		[26:0]	pclk_led_cnt;		
wire			smlh_link_up; 	
wire			rdlh_link_up/*synthesis PAP_MARK_DEBUG="1"*/; 	

// Uart to APB 32bits（原有信号，不修改）
wire			uart_p_sel;			
wire	[3:0]	uart_p_strb;		
wire	[15:0]	uart_p_addr;		
wire	[31:0]	uart_p_wdata;		
wire			uart_p_ce;			
wire			uart_p_we;			
wire			uart_p_rdy;			
wire	[31:0]	uart_p_rdata;		

// APB signal（原有信号，不修改）
wire	[3:0]	p_strb; 			
wire	[15:0]	p_addr; 			
wire	[31:0]	p_wdata; 			
wire			p_ce; 				
wire			p_we; 				


wire			p_sel_pcie;			
wire			p_sel_cfg;			
wire			p_sel_dma;			

wire	[31:0]	p_rdata_pcie;		
wire	[31:0]	p_rdata_cfg;		
wire	[31:0]	p_rdata_dma;		

wire			p_rdy_pcie;			
wire			p_rdy_cfg;			
wire			p_rdy_dma;			

// 原有赋值（不修改）
assign cfg_ido_req_en	=	1'b0;	
assign cfg_ido_cpl_en	=	1'b0;	
assign xadm_ph_cdts		=	8'b0;	
assign xadm_pd_cdts		=	12'b0;	
assign xadm_nph_cdts	=	8'b0;	
assign xadm_npd_cdts	=	12'b0;	
assign xadm_cplh_cdts	=	8'b0;	
assign xadm_cpld_cdts	=	12'b0;	

// Rst debounce（原有模块，不修改）
hsst_rst_cross_sync_v1_0 #(
	.RST_CNTR_VALUE		(16'hC000)
) u_refclk_buttonrstn_debounce (
	.clk				(ref_clk),			
	.rstn_in			(button_rst_n), 	
	.rstn_out			(sync_button_rst_n) 
);

hsst_rst_cross_sync_v1_0 #(
	.RST_CNTR_VALUE		(16'hC000)
) u_refclk_perstn_debounce (
	.clk				(ref_clk), 			
	.rstn_in			(perst_n),			
	.rstn_out			(sync_perst_n)		// 此处使用补充声明的sync_perst_n
);

hsst_rst_sync_v1_0  u_ref_core_rstn_sync (
	.clk				(ref_clk), 			
	.rst_n				(core_rst_n),		
	.sig_async			(1'b1),
	.sig_synced			(ref_core_rst_n)	
);

hsst_rst_sync_v1_0  u_pclk_core_rstn_sync (
	.clk				(pclk),				
	.rst_n				(core_rst_n),		
	.sig_async			(1'b1),
	.sig_synced			(s_pclk_rstn)		
);

always @(posedge ref_clk or negedge sync_perst_n) begin
	if (!sync_perst_n) begin
		ref_led_cnt <= 23'd0;
		ref_led <= 1'b1;
	end else if ( (smlh_link_up & rdlh_link_up) && (   fifo_full == 1'b1) ) begin
		ref_led_cnt <= ref_led_cnt + 23'd1;
		if(&ref_led_cnt)
			ref_led <= ~ref_led;
	end
end
//右边
always @(posedge pclk or negedge s_pclk_rstn) begin
	if (!s_pclk_rstn) begin
		pclk_led_cnt <= 27'd0;
		pclk_led <= 1'b1;
	end else if (smlh_link_up & rdlh_link_up)   begin
		pclk_led_cnt <= pclk_led_cnt + 27'd1;
		if(&pclk_led_cnt)
			pclk_led <= ~pclk_led;
	end
end
// axis_master_tuser[5:4];
// UART TO APB（原有模块，不修改）
pgr_uart2apb_top_32bit #(
	.CLK_DIV_P		(16'd145)
) u_uart2apb_top (
	.clk			(ref_clk),					
	.rst_n			(ref_core_rst_n),			
	.txd			(txd),						
	.rxd			(rxd),						
	.p_sel			(uart_p_sel),				
	.p_strb			(uart_p_strb),				
	.p_addr			(uart_p_addr),				
	.p_wdata		(uart_p_wdata),				
	.p_ce			(uart_p_ce),				
	.p_we			(uart_p_we),				
	.p_rdy			(uart_p_rdy),				
	.p_rdata		(uart_p_rdata)				
);

// APB MUX（原有模块，不修改）
ips2l_expd_apb_mux u_ips2l_pcie_expd_apb_mux (
	// From ref_clk domain
	.i_uart_clk				(ref_clk),			
	.i_uart_rst_n			(ref_core_rst_n),	
	.i_uart_p_sel			(uart_p_sel),		
	.i_uart_p_strb			(uart_p_strb),		
	.i_uart_p_addr			(uart_p_addr),		
	.i_uart_p_wdata			(uart_p_wdata),		
	.i_uart_p_ce			(uart_p_ce),		
	.i_uart_p_we			(uart_p_we),		
	.o_uart_p_rdy			(uart_p_rdy),		
	.o_uart_p_rdata			(uart_p_rdata),		
	// To pclk_div2 clock domain
	.i_pclk_div2_clk		(pclk_div2),		
	.i_pclk_div2_rst_n		(core_rst_n),		

	.o_pclk_div2_p_strb		(p_strb),			
	.o_pclk_div2_p_addr		(p_addr),			
	.o_pclk_div2_p_wdata	(p_wdata),			
	.o_pclk_div2_p_ce		(p_ce),				
	.o_pclk_div2_p_we		(p_we),				

	// To PCIe
	.o_pcie_p_sel			(p_sel_pcie),		
	.i_pcie_p_rdy			(p_rdy_pcie),		
	.i_pcie_p_rdata			(p_rdata_pcie),		

	// To DMA
	.o_dma_p_sel			(p_sel_dma),		
	.i_dma_p_rdy			(p_rdy_dma),		
	.i_dma_p_rdata			(p_rdata_dma),		

	// To config
	.o_cfg_p_sel			(p_sel_cfg),		
	.i_cfg_p_rdy			(p_rdy_cfg),		
	.i_cfg_p_rdata			(p_rdata_cfg)		
);

// DMA CTRL      BASE ADDR = 0x8000（原有模块，不修改）

ips2l_pcie_dma #(
	.DEVICE_TYPE			(DEVICE_TYPE),
	.AXIS_SLAVE_NUM			(AXIS_SLAVE_NUM)
) u_ips2l_pcie_dma (
	.clk					(pclk_div2),				
	.rst_n					(core_rst_n),				

	// Num
	.i_cfg_pbus_num			(cfg_pbus_num),				
	.i_cfg_pbus_dev_num		(cfg_pbus_dev_num),			
	.i_cfg_max_rd_req_size	(cfg_max_rd_req_size),		
	.i_cfg_max_payload_size	(cfg_max_payload_size),		

	// AXI4-Stream master interface
	.i_axis_master_tvld		(axis_master_tvalid_mem),	
	.o_axis_master_trdy		(axis_master_tready_mem),	
	.i_axis_master_tdata	(axis_master_tdata_mem),	
	.i_axis_master_tkeep	(axis_master_tkeep_mem),	
														
	.i_axis_master_tlast	(axis_master_tlast_mem),	
	.i_axis_master_tuser	(axis_master_tuser_mem),	

	// AXI4-Stream slave0 interface  
	.i_axis_slave0_trdy		(axis_slave0_tready),		
	.o_axis_slave0_tvld		(dma_axis_slave0_tvalid),	
	.o_axis_slave0_tdata	(dma_axis_slave0_tdata),	
	.o_axis_slave0_tlast	(dma_axis_slave0_tlast),	
	.o_axis_slave0_tuser	(dma_axis_slave0_tuser),	

	// AXI4-Stream slave1 interface（未使用）
	.i_axis_slave1_trdy		(axis_slave1_tready),		
	.o_axis_slave1_tvld		(axis_slave1_tvalid),		
	.o_axis_slave1_tdata	(axis_slave1_tdata),		
	.o_axis_slave1_tlast	(axis_slave1_tlast),		
	.o_axis_slave1_tuser	(axis_slave1_tuser),		

	// AXI4-Stream slave2 interface（未使用）
	.i_axis_slave2_trdy		(axis_slave2_tready),		
	.o_axis_slave2_tvld		(axis_slave2_tvalid),		
	.o_axis_slave2_tdata	(axis_slave2_tdata),		
	.o_axis_slave2_tlast	(axis_slave2_tlast),		
	.o_axis_slave2_tuser	(axis_slave2_tuser),		

	// From pcie
	.i_cfg_ido_req_en		(cfg_ido_req_en),			
	.i_cfg_ido_cpl_en		(cfg_ido_cpl_en),			
	.i_xadm_ph_cdts			(xadm_ph_cdts),				
	.i_xadm_pd_cdts			(xadm_pd_cdts),				
	.i_xadm_nph_cdts		(xadm_nph_cdts),			
	.i_xadm_npd_cdts		(xadm_npd_cdts),			
	.i_xadm_cplh_cdts		(xadm_cplh_cdts),			
	.i_xadm_cpld_cdts		(xadm_cpld_cdts),			

	// APB interface（配置DMA参数）
	.i_apb_psel				(p_sel_dma),				
	.i_apb_paddr			(p_addr[8:0]),				
	.i_apb_pwdata			(p_wdata),					
	.i_apb_pstrb			(p_strb),					
	.i_apb_pwrite			(p_we),						
	.i_apb_penable			(p_ce),						
	.o_apb_prdy				(p_rdy_dma),				
	.o_apb_prdata			(p_rdata_dma),				
	.o_cross_4kb_boundary	(cross_4kb_boundary),		//4k边界

//**************************图像测试********************************************
    .i_bar0_wr_clk		    (clk_100m ),		
	.i_bar0_wr_rst		    (rst_n  ),		
    .i_bar0_wr_en_text		(bar0_wr_en),			// BAR0写使能（1=写）
	.i_bar0_wr_addr_text    (bar0_wr_addr),			// BAR0写地址（12位，0~4095）
	.i_bar0_wr_data_text	(bar0_wr_data),			// BAR0写数据（128bit RGB565）
	.i_bar0_wr_byte_en_text	(bar0_wr_byte_en),	    // BAR0字节使能（16位全1，所有字节有效）
    .cpld_last_data         (cpld_last_data         ),
    .bar0_rd_clk_en         (bar0_rd_clk_en             ),
    .bar0_rd_addr           (bar0_rd_addr               )
);

//==========================================================================
// 新增：720p测试图案生成模块


// ========== 修改处1：参数适配RGB565 ==========
localparam  H_ACTIVE = 11'd1280;  // 720p水平有效像素（1280列，不变）
localparam  V_ACTIVE = 10'd720;   // 720p垂直有效像素（720行，不变）
localparam  PIXEL_BIT = 16;         // RGB565每个像素16bit
localparam  AXIS_WIDTH = 128;       // AXI数据位宽128bit
localparam  AXIS_BEAT_PIXELS = AXIS_WIDTH / PIXEL_BIT; // 每拍像素数：128/16=8

localparam  TOTAL_PIXELS_PER_FRAME = H_ACTIVE * V_ACTIVE; // 总像素：1280×720=921600
localparam  TOTAL_BEATS_PER_FRAME = TOTAL_PIXELS_PER_FRAME / AXIS_BEAT_PIXELS; // 总拍数：921600/8=115200

// ========== 修改处2：RGB565颜色定义（16位，R5G6B5） ==========
localparam  COLOR_RED = 16'hFFFF;  //F800;    // 全红：R=5'h1F, G=6'h00, B=5'h00 → 16'hF800    11111 000000 00000
localparam  COLOR_BLUE = 16'h0000;  //001F;   // 全蓝：R=5'h00, G=6'h00, B=5'h1F → 16'h001F    00000 000000 11111
localparam  COLOR_GREEN = 16'h07E0;  // 全绿：R=5'h00, G=6'h3F, B=5'h00 → 16'h07E0    00000 111111 00000
reg [1:0]  color_state;    // 颜色状态（0=红，1=蓝，2=绿）

// 新增：分块循环状态机定义（4个状态）
localparam IDLE = 2'd0;
localparam WRITE = 2'd1;
localparam WAIT_READ = 2'd2;
localparam UPDATE = 2'd3;
reg [1:0] blk_state/*synthesis PAP_MARK_DEBUG="1"*/; // 分块状态寄存器
reg [17:0] pixel_offset; // 像素偏移（记录当前块的起始像素，如第1块=0，第2块=4096）
// ========== 新增：BAR0 RAM写接口（对接测试图生成模块） ==========
reg				bar0_wr_en;					// BAR0写使能（1=写）
reg		[11:0]	bar0_wr_addr/*synthesis PAP_MARK_DEBUG="1"*/;				// BAR0写地址（12位，0~4095）
reg		[127:0]	bar0_wr_data;				// BAR0写数据（128bit RGB565）
reg		[15:0]	bar0_wr_byte_en;				// BAR0字节使能（16位全1，所有字节有效）
// 新增寄存器：保存下一个要写入的地址
reg [11:0] next_wr_addr;
reg		[16:0]	beat_cnt;		// 每帧传输拍数计数（0~115199）
reg				frame_done;		// 一帧生成完成标志
wire               cpld_last_data/*synthesis PAP_MARK_DEBUG="1"*/;  

// 2. 自增计数器：分别控制低八位和高八位（0~255，对应8位自增范围）
reg [7:0] low_byte_cnt;  // 第一种数据：低八位计数（0x00~0xFF）
reg [7:0] high_byte_cnt; // 第二种数据：高八位计数（0x00~0xFF）



reg ddr_fifo_rd_en_d1;  // 延迟1周期的FIFO读使能（对齐RAM写时序）

// 延迟逻辑：在clk_100m上升沿采样，延迟1周期
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) begin
        ddr_fifo_rd_en_d1 <= 1'b0;
    end else begin
        ddr_fifo_rd_en_d1 <= ddr_fifo_rd_en;  // t0时刻的rd_en，t1时刻出现在d1
    end
end
// 定义寄存器存储上一次的ddr_fifo_rd_data（用于对比[31:24]位）
reg [127:0] ddr_fifo_rd_data_prev;
always @(posedge clk_100m or negedge rst_n) begin  // 注意：时钟需与blk_state所在时钟域一致
    if (!rst_n) begin
        ddr_fifo_rd_data_prev <= 128'd0;
    end else begin
        // 每个时钟周期更新上一次的数据（确保对比的是“上一拍”的值）
        ddr_fifo_rd_data_prev <= ddr_fifo_rd_data;
    end
end
reg cmos1_vsync_pos_1;//摄像头1图片结束
// -------------------------- 新增：跨时钟域同步与边沿检测信号 --------------------------
// 1. cmos1_vsync_pos_1（clk_100m域）→ ddr_core_clk域同步寄存器（两级防亚稳态）
reg [1:0] sync_cmos1_vsync_pos_1;  // 同步后信号（[1]为稳定值，[0]为中间值）
// 2. 边沿检测延迟寄存器（用于捕捉下降沿）
reg cmos1_vsync_pos_1_dly;
// 3. 同步后的cmos1_vsync_pos_1下降沿信号（仅1周期高，触发一次）
wire sync_cmos1_vsync_pos_1_neg;
// -------------------------- 同步：clk_100m→ddr_core_clk --------------------------
always @(posedge ddr_core_clk or negedge global_rst_n) begin
    if (!global_rst_n) begin
        sync_cmos1_vsync_pos_1 <= 2'b0;  // 复位清零
    end else begin
        // 两级同步：确保信号在ddr_core_clk域稳定
        sync_cmos1_vsync_pos_1 <= {sync_cmos1_vsync_pos_1[0], cmos1_vsync_pos_1};
    end
end

// -------------------------- 检测：cmos1_vsync_pos_1下降沿 --------------------------
always @(posedge ddr_core_clk or negedge global_rst_n) begin
    if (!global_rst_n) begin
        cmos1_vsync_pos_1_dly <= 1'b0;  // 复位清零
    end else begin
        // 延迟1周期，用于对比当前与前一周期信号
        cmos1_vsync_pos_1_dly <= sync_cmos1_vsync_pos_1[1];
    end
end
// 下降沿判定：当前周期为0，前一周期为1（仅1周期高）
assign sync_cmos1_vsync_pos_1_neg = !sync_cmos1_vsync_pos_1[1] && cmos1_vsync_pos_1_dly;

// 定义用于标记检测到8'hFE的寄存器
reg has_fe;
// 定义用于检测cmos1_vsync_pos_1下降沿的延迟寄存器（clk_100m域）
reg cmos1_vsync_pos_1_dly_fe;

// 核心逻辑：检测8'hFE并响应cmos1_vsync_pos_1下降沿
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) begin
        // 复位时清零所有寄存器
        has_fe <= 1'b0;
        cmos1_vsync_pos_1_dly_fe <= 1'b0;
    end else begin
        // 1. 延迟寄存cmos1_vsync_pos_1，用于检测下降沿
        cmos1_vsync_pos_1_dly_fe <= cmos1_vsync_pos_1;
        
        // 2. 优先响应cmos1_vsync_pos_1的下降沿（清零has_fe）
        if (!cmos1_vsync_pos_1 && cmos1_vsync_pos_1_dly_fe) begin  // 下降沿：当前0，上一拍1
            has_fe <= 1'b0;
        end 
        // 3. 若未检测到下降沿，检查是否有8'hFE（有则置1，否则保持原状态）
        else if ( (ddr_fifo_rd_data[31:24]  == 8'hFE)  || 
                  (ddr_fifo_rd_data[63:56]  == 8'hFE)  || 
                  (ddr_fifo_rd_data[95:88]  == 8'hFE)  || 
                  (ddr_fifo_rd_data[127:120] == 8'hFE) ) begin
            has_fe <= 1'b1;
        end
        // 4. 其他情况保持has_fe不变（避免无8'hFE时误清零）
        else begin
            has_fe <= has_fe;
        end
    end
end
reg ddr_fifo_rd_en_stop;

reg ddr_fifo_rd_en_early;
reg ddr_fifo_rd_en_early_reg;
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) begin
        ddr_fifo_rd_en_early <= 1'b0;   
        ddr_fifo_rd_en_early_reg <= 1'b1; 
    end else begin        
        if (ddr_fifo_rd_en_early_reg == 1'b1 && !ddr_almost_empty) begin
              ddr_fifo_rd_en_early <= 1'b1;
              ddr_fifo_rd_en_early_reg <= 1'b0; 
        end else begin
              ddr_fifo_rd_en_early <= 1'b0;
        end
    end
end
reg ddr_fifo_rd_en_pcie;
// 分块状态机（核心修改：bar0_wr_data=DDR3读数据）
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) begin
         blk_state <= IDLE;
        //pixel_offset <= 18'd0; // 初始偏移0（第1块从0像素开始）
        bar0_wr_en <= 1'b0;
        bar0_wr_addr <= 12'd0;
        next_wr_addr <= 12'd0;      // 下一个地址（初始1）
        bar0_wr_data <= 128'd0;
        bar0_wr_byte_en <= 16'h0000;
        //wr_cnt <= 1'b0; // 复位为准备阶段
        //IIC
        Reg0_wr_en <= 1'b0; 
        Reg1_wr_en <= 1'b0; 
        myReg0_w <= 8'd1;
        myReg1_w <= 8'd1;
        Reg_wr_state <= 1'b0; 
        //low_byte_cnt <= 8'h00;      // 重置计数器
        //high_byte_cnt <= 8'h00;     // 重置计数器
        cmos1_vsync_pos_1 <= 1'b0;
        ddr_fifo_rd_en_stop <=1'b0;
        ddr_fifo_rd_en_pcie <= 1'b0;
    end else if(  ddr_init_done ) begin// 新增DDR3就绪判断
        case (blk_state)
            IDLE: begin
                bar0_wr_en <= 1'b0;
                ddr_fifo_rd_en_pcie <= 1'b0;
                bar0_wr_addr <= 12'd0;
                next_wr_addr <= 12'd0;
                ddr_fifo_rd_en_stop <=1'b0;
                // 等待IIC控制信号
                //if((Reg_wr_state == 1'b0)&& ((myReg0_flag == 8'd2 )||(myReg0_flag == 8'd0) )) begin
                if((Reg_wr_state == 1'b0)&& (myReg0_flag == 8'd2 )) begin
                    blk_state <= WRITE;
                    Reg1_wr_en <= 1'b0; 
                    //cmos1_vsync_pos_1 <= 1'b0;
                end else if((Reg_wr_state == 1'b1)&& (myReg1_flag == 8'd2)) begin
                    blk_state <= WRITE;
                    Reg0_wr_en <= 1'b0;
                    //cmos1_vsync_pos_1 <= 1'b0;
                end
            end

            WRITE: begin
                bar0_wr_byte_en <= 16'hFFFF; // 128bit全有效
                if (bar0_wr_addr == 12'd4095) begin
                    bar0_wr_en <= 1'b0;
                    ddr_fifo_rd_en_pcie <= 1'b0;
                    blk_state <= WAIT_READ;
                    cmos1_vsync_pos_1 <= 1'b0;
                 // 读使能=新FIFO非空（有同步后的数据）
                end else if (!ddr_almost_empty && ddr_fifo_rd_en_early_reg ==1'b0) begin  // FIFO有数据可读  
         
                    bar0_wr_en <= 1'b1;   
                    bar0_wr_addr <= next_wr_addr;
                    //bar0_wr_addr <= bar0_wr_addr+12'd1;
                    next_wr_addr <= next_wr_addr + 12'd1;
                    bar0_wr_data <= ddr_fifo_rd_data;
                    ddr_fifo_rd_en_pcie <= 1'b1; 

    
         
                    if(ddr_fifo_rd_data[31:24] == 8'hFD ) begin
                        cmos1_vsync_pos_1 <= 1'b1;
                    end

                end else begin
                    bar0_wr_en <= 1'b0;   
                    ddr_fifo_rd_en_pcie <= 1'b0;                    
                end
 
            end
            WAIT_READ: begin
                if (Reg_wr_state == 1'b0 ) begin
                    Reg0_wr_en <= 1'b1; 
                    Reg_wr_state <= 1'b1;
                    blk_state <= UPDATE;
                end else begin
                    Reg1_wr_en <= 1'b1; 
                    Reg_wr_state <= 1'b0;
                    blk_state <= UPDATE;
                end      
            end

            UPDATE: begin
                blk_state <= IDLE;
            end
        endcase
    end
end
//==========================================================================
// 新增结束
//==========================================================================

// CFG CTRL（EP模式下不实例化，保持原有代码不变）
generate
	if (DEVICE_TYPE == 3'd4) begin:rc
	//CFG TLP TX RX     BASE ADDR = 0x9000
		pcie_cfg_ctrl u_pcie_cfg_ctrl (
			//from APB
			.pclk_div2				(pclk_div2),				//125mhz    x2 5gt/s
			.apb_rst_n				(core_rst_n),				
			.p_sel					(p_sel_cfg),				
			.p_strb					(p_strb),					
			.p_addr					(p_addr[7:0]),				
			.p_wdata				(p_wdata),					
			.p_ce					(p_ce),						
			.p_we					(p_we),						
			.p_rdy					(p_rdy_cfg),				
			.p_rdata				(p_rdata_cfg),				
			.pcie_cfg_ctrl_en		(pcie_cfg_ctrl_en),			

			//To PCIE ctrl
			.axis_slave_tready		(axis_slave0_tready),		
			.axis_slave_tvalid		(cfg_axis_slave0_tvalid),	
			.axis_slave_tlast		(cfg_axis_slave0_tlast),	
			.axis_slave_tuser		(cfg_axis_slave0_tuser),	
			.axis_slave_tdata		(cfg_axis_slave0_tdata),	

			.axis_master_tready		(axis_master_tready_cfg),	
			.axis_master_tvalid		(axis_master_tvalid),		
			.axis_master_tlast		(axis_master_tlast),		

			.axis_master_tkeep		(axis_master_tkeep),		

			.axis_master_tdata		(axis_master_tdata)			
		);

		// Logic mux
		assign axis_slave0_tvalid      = pcie_cfg_ctrl_en ? cfg_axis_slave0_tvalid  : dma_axis_slave0_tvalid;
		assign axis_slave0_tlast       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tlast   : dma_axis_slave0_tlast;
		assign axis_slave0_tuser       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tuser   : dma_axis_slave0_tuser;
		assign axis_slave0_tdata       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tdata   : dma_axis_slave0_tdata;

		assign axis_master_tvalid_mem  = pcie_cfg_ctrl_en ? 1'b0                    : axis_master_tvalid;
		assign axis_master_tdata_mem   = pcie_cfg_ctrl_en ? 128'b0                  : axis_master_tdata;
		assign axis_master_tkeep_mem   = pcie_cfg_ctrl_en ? 4'b0                    : axis_master_tkeep;
		assign axis_master_tlast_mem   = pcie_cfg_ctrl_en ? 1'b0                    : axis_master_tlast;
		assign axis_master_tuser_mem   = pcie_cfg_ctrl_en ? 8'b0                    : axis_master_tuser;

		assign axis_master_tready      = pcie_cfg_ctrl_en ? axis_master_tready_cfg  : axis_master_tready_mem;
	end else begin:ep
		assign p_rdy_cfg               = 1'b0;
		assign p_rdata_cfg             = 32'b0;

		
		assign axis_slave0_tvalid      = dma_axis_slave0_tvalid;
		assign axis_slave0_tlast       = dma_axis_slave0_tlast;
		assign axis_slave0_tuser       = dma_axis_slave0_tuser;
		assign axis_slave0_tdata       = dma_axis_slave0_tdata;

		assign axis_master_tvalid_mem  = axis_master_tvalid;
		assign axis_master_tdata_mem   = axis_master_tdata;
		assign axis_master_tkeep_mem   = axis_master_tkeep;
		assign axis_master_tlast_mem   = axis_master_tlast;
		assign axis_master_tuser_mem   = axis_master_tuser;

		assign axis_master_tready      = axis_master_tready_mem;
	end
endgenerate

// PCIe IP TOP : HSSTLP : 0x0000~6000 PCIe BASE ADDR : 0x7000（原有模块，不修改）
pcie_test u_ips2l_pcie_wrap (
	.button_rst_n				(sync_button_rst_n),	
	.power_up_rst_n				(sync_perst_n),			
	.perst_n					(sync_perst_n),			

	// The clock and reset signals
	.pclk						(pclk),					
	.pclk_div2					(pclk_div2),			
	.ref_clk					(ref_clk),				
	.ref_clk_n					(ref_clk_n),			
	.ref_clk_p					(ref_clk_p),			
	.core_rst_n					(core_rst_n),			

	// APB interface to DBI config
	.p_sel						(p_sel_pcie),			
	.p_strb						(uart_p_strb),			
	.p_addr						(uart_p_addr),			
	.p_wdata					(uart_p_wdata),			
	.p_ce						(uart_p_ce),			
	.p_we						(uart_p_we),			
	.p_rdy						(p_rdy_pcie),			
	.p_rdata					(p_rdata_pcie),			

	// PHY diff signals
	.rxn						(rxn),					
	.rxp						(rxp),					
	.txn						(txn),					
	.txp						(txp),					
	.pcs_nearend_loop			({4{1'b0}}),			
	.pma_nearend_ploop			({4{1'b0}}),			
	.pma_nearend_sloop			({4{1'b0}}),			

	// AXI4-Stream master interface
	.axis_master_tvalid			(axis_master_tvalid),	
	.axis_master_tready			(axis_master_tready),	
	.axis_master_tdata			(axis_master_tdata),	
	.axis_master_tkeep			(axis_master_tkeep),	
														
	.axis_master_tlast			(axis_master_tlast),	
	.axis_master_tuser			(axis_master_tuser),	

	// AXI4-Stream slave 0 interface
	.axis_slave0_tready			(axis_slave0_tready),	
	.axis_slave0_tvalid			(axis_slave0_tvalid),	
	.axis_slave0_tdata			(axis_slave0_tdata),	
	.axis_slave0_tlast			(axis_slave0_tlast),	
	.axis_slave0_tuser			(axis_slave0_tuser),	

	// AXI4-Stream slave 1 interface（未使用）
	.axis_slave1_tready			(axis_slave1_tready),	
	.axis_slave1_tvalid			(axis_slave1_tvalid),	
	.axis_slave1_tdata			(axis_slave1_tdata),	
	.axis_slave1_tlast			(axis_slave1_tlast),	
	.axis_slave1_tuser			(axis_slave1_tuser),	

	// AXI4-Stream slave 2 interface（未使用）
	.axis_slave2_tready			(axis_slave2_tready),	
	.axis_slave2_tvalid			(axis_slave2_tvalid),	
	.axis_slave2_tdata			(axis_slave2_tdata),	
	.axis_slave2_tlast			(axis_slave2_tlast),	
	.axis_slave2_tuser			(axis_slave2_tuser),	

	.pm_xtlh_block_tlp			(),						

	.cfg_send_cor_err_mux		(),						
	.cfg_send_nf_err_mux		(),						
	.cfg_send_f_err_mux			(),						
	.cfg_sys_err_rc				(),						
	.cfg_aer_rc_err_mux			(),						

	// The radm timeout
	.radm_cpl_timeout			(),						

	// Configuration signals
	.cfg_max_rd_req_size		(cfg_max_rd_req_size),	
	.cfg_bus_master_en			(),						
	.cfg_max_payload_size		(cfg_max_payload_size),	
	.cfg_ext_tag_en				(),						
	.cfg_rcb					(cfg_rcb),				
	.cfg_mem_space_en			(),						
	.cfg_pm_no_soft_rst			(),						
	.cfg_crs_sw_vis_en			(),						
	.cfg_no_snoop_en			(),						
	.cfg_relax_order_en			(),						
	.cfg_tph_req_en				(),						
	.cfg_pf_tph_st_mode			(),						
	.rbar_ctrl_update			(),						
	.cfg_atomic_req_en			(),						

	.cfg_pbus_num				(cfg_pbus_num),			
	.cfg_pbus_dev_num			(cfg_pbus_dev_num),		

	// Debug signals
	.radm_idle					(),						
	.radm_q_not_empty			(),						
	.radm_qoverflow				(),						
	.diag_ctrl_bus				(2'b0),					
	.cfg_link_auto_bw_mux		(),						
	.cfg_bw_mgt_mux				(),						
	.cfg_pme_mux				(),						
	.app_ras_des_sd_hold_ltssm	(1'b0),					
	.app_ras_des_tba_ctrl		(2'b0),					

	.dyn_debug_info_sel			(4'b0),					
	.debug_info_mux				(),

	// System signal
	.smlh_link_up				(smlh_link_up),			//link状态
	.rdlh_link_up				(rdlh_link_up),			//link状态
	.smlh_ltssm_state			(smlh_ltssm_state)
);
     


endmodule