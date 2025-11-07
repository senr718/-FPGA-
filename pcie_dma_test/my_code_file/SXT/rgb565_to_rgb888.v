module rgb565_to_rgb888 (
    input                  rst_n,       // 输入复位（组合逻辑下仅用于复位时清零）
    input        [15:0]    i_rgb565,    // 输入RGB565数据
    output       [23:0]    o_rgb888    // 输出RGB888数据（R[23:16],G[15:8],B[7:0]）

);

// 组合逻辑实现：无时钟延迟，输出与输入同步变化
assign o_rgb888 = (!rst_n) ? 24'd0 : 
                 {i_rgb565[15:11], 3'b000,  // R: 5bit→8bit（补3个0）
                  i_rgb565[10:5],  2'b00,   // G: 6bit→8bit（补2个0）
                  i_rgb565[4:0],   3'b000}; // B: 5bit→8bit（补3个0）

endmodule