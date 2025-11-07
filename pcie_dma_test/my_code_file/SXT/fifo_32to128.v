module fifo_32to128 (
    input               clk_100m,        // 读时钟
    input               rst_n,           // 全局复位（高有效）
    input        [31:0] fifo_rd_data,    // FIFO输出的32bit数据
    input               fifo_empty,      // FIFO空标志（低电平非空）
    input               blk_state_WRITE, // 外部WRITE状态（高有效）
    output reg          fifo_rd_en,      // FIFO读使能
    output reg  [127:0] data_128bit,     // 拼接后的128bit数据
    output reg          data_128_valid   // 128bit数据有效标志
);

reg [1:0] cnt_read;                     // 读取计数器（0~3），FIFO空时保持当前值
reg [1:0] next_cnt_read;  
reg [31:0] data_reg [3:0];              // 数据暂存寄存器，FIFO空时保留数据
localparam IDLE  = 1'b0, READ = 1'b1;
reg current_state, next_state;

// 状态机跳转：仅在“读完4个数据”或“退出WRITE状态”时跳回IDLE，FIFO空时保持READ
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) current_state <= IDLE;
    else current_state <= next_state;
end

// 状态机次态逻辑：FIFO空时不退出READ，仅暂停读取
always @(*) begin
    case (current_state)
        IDLE: begin
            // 仅当WRITE状态且FIFO非空时，进入READ
            next_state = (blk_state_WRITE && !fifo_empty) ? READ : IDLE;
        end
        READ: begin
            // 退出READ的唯一条件：读完4个数据 或 退出WRITE状态（与FIFO空无关）
            next_state = (cnt_read == 2'd3 || !blk_state_WRITE) ? IDLE : READ;
        end
        default: next_state = IDLE;
    endcase
end

// 读使能：仅在READ状态、FIFO非空、WRITE状态时有效（FIFO空时自动为0，暂停读取）
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) fifo_rd_en <= 1'b0;
    else begin
        fifo_rd_en <= (current_state == READ) && !fifo_empty && blk_state_WRITE;
    end
end

// 计数器：仅在有效读时递增，FIFO空时保持当前值，退出WRITE时清零
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) begin
        cnt_read <= 2'd0;
        next_cnt_read <= 2'd0;
    end else if (!blk_state_WRITE) begin  // 退出WRITE状态，强制清零（彻底终止）
        cnt_read <= 2'd0;
        next_cnt_read <= 2'd0;
    end else if (fifo_rd_en) begin  // 有效读时计数递增（FIFO非空且在READ状态）
        cnt_read <= next_cnt_read + 2'd1;
        next_cnt_read <= next_cnt_read + 2'd1;
    end
    // 隐含逻辑：FIFO空时（fifo_rd_en=0），cnt_read保持当前值（不清零）
end

// 数据寄存器：FIFO空时保留数据，有效读时存储，退出WRITE时清零
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) begin
        data_reg[0] <= 32'd0;
        data_reg[1] <= 32'd0;
        data_reg[2] <= 32'd0;
        data_reg[3] <= 32'd0;
    end else if (!blk_state_WRITE) begin  // 退出WRITE状态，清零数据（彻底终止）
        data_reg[0] <= 32'd0;
        data_reg[1] <= 32'd0;
        data_reg[2] <= 32'd0;
        data_reg[3] <= 32'd0;
    end else if (fifo_rd_en) begin  // 有效读时存储数据（FIFO非空时继续填充）
        case (cnt_read)
            2'd0: data_reg[0] <= fifo_rd_data;
            2'd1: data_reg[1] <= fifo_rd_data;
            2'd2: data_reg[2] <= fifo_rd_data;
            2'd3: data_reg[3] <= fifo_rd_data;
        endcase
    end
    // 隐含逻辑：FIFO空时，data_reg保持已存数据（不清除）
end

// 128bit数据有效：仅当4个数据全部读完时有效
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) begin
        data_128bit <= 128'd0;
        data_128_valid <= 1'b0;
    end else begin
        // 当读完第4个数据（cnt_read=3且读使能有效），输出128bit数据
        if (cnt_read == 2'd3 && fifo_rd_en) begin
            data_128bit <= {data_reg[3], data_reg[2], data_reg[1], data_reg[0]};
            data_128_valid <= 1'b1;
        end else begin
            data_128_valid <= 1'b0;
        end
    end
end

endmodule