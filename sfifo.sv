///////////////////////////////////////////////////////////////////////////
// Copyright 2023 DreamBig Semiconductor, Inc. All Rights Reserved.
//
// No portions of this material may be reproduced in any form without
// the written permission of DreamBig Semiconductor Inc.
// All information contained in this document is DreamBig Semiconductor Inc.
// company confidential, proprietary and trade secret.
//
/// Author: Aaron Rother
/// Date Created: 2021
/// Description: Parameterized, Synchronous FIFO
/// - Data flops may be configured as reset or no-reset
/// - Parameterized empty and full thresholds
/// - Number of valid entries output
/// - Underflow/Overflow indicators
/// - Combinational or sequential read data.
///   - If combinational, read data are driven from a MUX. This is for cases in which read data timing is not critical.
///   - If sequential, read data are driven from flops. This provides the best read data timing, but with /a
///     trade-off of slightly worse input control signal timing.
/// - Flush strobe; if asserted, resets all pointers; takes precedence over all other control signals
///////////////////////////////////////////////////////////////////////////

module sfifo #(parameter DEPTH = 4,       // FIFO Depth >=1
               parameter WIDTH = 4,       // FIFO Width >=1
               parameter FTHRESH = DEPTH, // Threshold FIFO valid entry count above which tfull is true.
                                          // Examples:   DEPTH: tfull == full
                                          //                 1: tfull == !empty
               parameter ETHRESH = 0,     // Threshold FIFO valid entry count below which tempty is true.
                                          // Examples:       0: tempty == empty
                                          //           DEPTH-1: tempty == !full
               parameter NO_DATA_RST = 0, // 0: Reset all flops. 1: Reset control but not data flops
               parameter RD_MUX = 0,      // 0: Read data driven from flop. 1: Read data driven combinationally from FIFO array MUX
                // Don't Override
               parameter NV_WIDTH = (((DEPTH == 1) ? 1 : $clog2(DEPTH)) + 1) ) // Num Valid Width. Always one more than number of bits needed for pointer
    (
     input  logic                clk,
     input  logic                rst_n,
     input  logic                flush,
     input  logic                wr,
     input  logic [WIDTH-1:0]    wr_data,
     input  logic                rd,
     output logic [WIDTH-1:0]    rd_data,
     output logic                full,
     output logic                tfull,
     output logic                empty,
     output logic                tempty,
     output logic                udf,
     output logic                ovf,
     output logic [NV_WIDTH-1:0] num_vld
     );

    //################################################################################
    // Declarations
    //################################################################################
    localparam DEPTH_FOR_PTR = RD_MUX ? DEPTH : ((DEPTH == 1) ? 1 : (DEPTH-1));
    localparam PTR_WIDTH = (DEPTH_FOR_PTR==1) ? 1 : $clog2(DEPTH_FOR_PTR);
    localparam [PTR_WIDTH-1:0] PTR_ONE = PTR_WIDTH'(1);
    localparam [NV_WIDTH-1:0] VLDS_ONE = NV_WIDTH'(1);

    logic [DEPTH-1:0] [WIDTH-1:0] fifo, fifo_nxt;
    logic [PTR_WIDTH-1:0] wr_ptr, rd_ptr, rd_ptr_nxt;
    logic [NV_WIDTH-1:0] num_vld_nxt;
    logic udf_nxt, ovf_nxt;

    //################################################################################
    // Pointers
    //################################################################################
    generate
        if (DEPTH == 1) begin: ptr_d_eq1
            assign wr_ptr = 1'b0;
            assign rd_ptr = 1'b0;
            assign rd_ptr_nxt = 1'b0;
        end: ptr_d_eq1

        else begin: ptr_d_gt1
            logic [PTR_WIDTH-1:0] wr_ptr_nxt;

            always_ff @ (posedge clk or negedge rst_n)
                if (~rst_n) begin
                    wr_ptr <= '0;
                    rd_ptr <= '0;
                end
                else begin
                    wr_ptr <= wr_ptr_nxt;
                    rd_ptr <= rd_ptr_nxt;
                end

            always_comb begin
                wr_ptr_nxt = wr_ptr;
                if (flush)
                  wr_ptr_nxt = '0;
                else if (wr)
                    wr_ptr_nxt = (wr_ptr == (DEPTH_FOR_PTR-1)) ? '0 : wr_ptr + PTR_ONE;
            end

            always_comb begin
                rd_ptr_nxt = rd_ptr;
                if (flush)
                  rd_ptr_nxt = '0;
                else if (rd)
                    rd_ptr_nxt = (rd_ptr == (DEPTH_FOR_PTR-1)) ? '0 : rd_ptr + PTR_ONE;
            end
        end: ptr_d_gt1
    endgenerate

    //################################################################################
    // Write and Read
    //################################################################################
    generate
        if (NO_DATA_RST) begin: data_rst_dis
            always_ff @ (posedge clk)
                fifo <= fifo_nxt;
        end: data_rst_dis
        else begin: data_rst_en
            always_ff @ (posedge clk or negedge rst_n)
                if (~rst_n) begin
                    fifo <= '0;
                end
                else begin
                    fifo <= fifo_nxt;
                end
        end: data_rst_en
    endgenerate

    generate
        // Read MUX Enable
        // This results in combinational logic in the read data path, proportional to log2(DEPTH).
        if (RD_MUX) begin: rd_mux_en

            // Write to pointer location
            always_comb begin
                fifo_nxt = fifo;
                if (wr) begin
                    for (int unsigned i = 0; i < DEPTH_FOR_PTR; i++) begin
                        if (wr_ptr == PTR_WIDTH'(i))
                            fifo_nxt[i] = wr_data;
                    end
                end
            end

            // MUX Read Data
            always_comb begin
                rd_data = '0;
                for (int unsigned i = 0; i < DEPTH_FOR_PTR; i++) begin
                    if (rd_ptr == PTR_WIDTH'(i))
                        rd_data = fifo[i];
                end
            end
        end: rd_mux_en

        // Read MUX Disable
        // Read data are driven from flops, with a trade-off of slightly worse input control signal timing.
        else begin: rd_mux_dis
            logic wrdata_to_rddata;

            always_comb begin
                fifo_nxt = fifo;
                if (wr) begin
                    for (int unsigned i = 0; i < DEPTH_FOR_PTR; i++) begin
                        if (wr_ptr == PTR_WIDTH'(i))
                            fifo_nxt[i] = wr_data;
                    end
                end

                if (wr & wrdata_to_rddata) begin
                    fifo_nxt[DEPTH-1] = wr_data;
                end
                else if (rd) begin
                    for (int unsigned i = 0; i < DEPTH_FOR_PTR; i++) begin
                        if (rd_ptr_nxt == PTR_WIDTH'(i))
                            fifo_nxt[DEPTH-1] = fifo[i];
                    end
                end
            end
            // FIFO output is fixed at FIFO[DEPTH-1]
            always_comb rd_data = fifo[DEPTH-1];

            always_comb wrdata_to_rddata = (~|num_vld) | (rd & (num_vld == VLDS_ONE));
        end: rd_mux_dis
    endgenerate

    //################################################################################
    // Status
    //################################################################################
    always_ff @ (posedge clk or negedge rst_n)
        if (~rst_n) begin
            full <= 1'b0;
            empty <= 1'b1;
            tfull <= 1'b0;
            tempty <= 1'b1;
            num_vld <= '0;
            udf <= 1'b0;
            ovf <= 1'b0;
        end
        else begin
            full <= num_vld_nxt == DEPTH;
            empty <= num_vld_nxt == '0;
            tfull <= num_vld_nxt >= FTHRESH;
            tempty <= num_vld_nxt <= ETHRESH;
            num_vld <= num_vld_nxt;
            udf <= udf_nxt;
            ovf <= ovf_nxt;
        end

    always_comb begin
        num_vld_nxt = num_vld;
        udf_nxt = udf;
        ovf_nxt = ovf;
        if (flush)
          num_vld_nxt = '0;
        else if (rd & ~wr)
            num_vld_nxt = num_vld - VLDS_ONE;
        else if (~rd & wr)
            num_vld_nxt = num_vld + VLDS_ONE;

        if (flush)
          udf_nxt = 1'b0;
        else if (rd &(num_vld == '0))
            udf_nxt = 1'b1;

        if (flush)
          ovf_nxt = 1'b0;
        else if (~rd & wr & (num_vld == DEPTH))
            ovf_nxt = 1'b1;
    end

`ifdef DBS_SIM_OR_FPV
    //################################################################################
    // Assertions
    //################################################################################
    default clocking def_clock @ (posedge clk); endclocking
    default disable iff (~rst_n);

 `ifdef DBS_SIM
    // Sim-only
    ast_fifo_ovf: assert property (!(ovf));
    ast_fifo_udf: assert property (!(udf));
    ast_fifo_depth: assert property (!(DEPTH==0));
    ast_fifo_fthresh: assert property (!(FTHRESH>DEPTH));
    ast_fifo_ethresh: assert property (!((ETHRESH==DEPTH) && (DEPTH>1)));
 `endif

 `ifdef DBS_FPV_LIB
    // Formal
    // Dead code in ptr next state equations expected under these parameter settings:
    // RD_MUX == 0, DEPTH == {1,2}
    // RD_MUX == 1, DEPTH == 1
    localparam PTR_WIDTH_FPV = (DEPTH==1) ? 1 : $clog2(DEPTH);

    bit [PTR_WIDTH_FPV-1:0] wr_ptr_fpv, rd_ptr_fpv, wr_ptr_prev, rd_ptr_prev;
    bit [DEPTH-1:0] [WIDTH-1:0] fifo_fpv;
    bit [WIDTH-1:0] rd_data_fpv;
    bit udf_sticky, ovf_sticky;

    always_ff @ (posedge clk or negedge rst_n)
        if (~rst_n) begin
            udf_sticky <= 1'b0;
            ovf_sticky <= 1'b0;
            wr_ptr_prev <= '0;
            rd_ptr_prev <= '0;
            wr_ptr_fpv <= '0;
            rd_ptr_fpv <= '0;
            fifo_fpv <= '0;
        end
        else begin
            udf_sticky <= udf | udf_sticky;
            ovf_sticky <= ovf | ovf_sticky;
            wr_ptr_prev <= wr_ptr;
            rd_ptr_prev <= rd_ptr;

            if (wr)
                wr_ptr_fpv <= (wr_ptr_fpv == DEPTH-1) ? '0 : wr_ptr_fpv + PTR_WIDTH_FPV'(1);
            else
                wr_ptr_fpv <= wr_ptr_fpv;

            if (rd)
                rd_ptr_fpv <= (rd_ptr_fpv == DEPTH-1) ? '0 : rd_ptr_fpv + PTR_WIDTH_FPV'(1);
            else
                rd_ptr_fpv <= rd_ptr_fpv;

            if (wr)
                fifo_fpv[wr_ptr_fpv] <= wr_data;
        end

    always_comb rd_data_fpv = fifo_fpv[rd_ptr_fpv];

    asm_flush: assume property (flush == 1'b0);

    ast_fifo_ovf: assert property (wr & ~rd & (num_vld == DEPTH) |=> ovf);
    ast_fifo_udf: assert property (rd & ~wr & (num_vld == 0) |=> udf);
    ast_fifo_full_empty: assert property (!(full && empty));
    ast_full: assert property (full == (num_vld == DEPTH));
    ast_empty: assert property (empty == (num_vld == 0));
    ast_tfull: assert property (tfull == (num_vld >= FTHRESH));
    ast_tempty: assert property (tempty == (num_vld <= ETHRESH));

    ast_rd_data: assert property (!empty && !udf && !udf_sticky && !ovf && !ovf_sticky |-> rd_data == rd_data_fpv);
    generate
        if (DEPTH==1) begin: depth_eq1
            ast_wr_ptr_nxt: assert property (!(wr_ptr != 0));
            ast_rd_ptr_nxt: assert property (!(rd_ptr != 0));
        end: depth_eq1
        else begin: depth_gt1
            if ((RD_MUX==1) || (DEPTH > 2)) begin: ptrs_depth_gt1
                ast_wr_ptr_nxt: assert property (wr |=> wr_ptr != wr_ptr_prev);
                ast_rd_ptr_nxt: assert property (rd |=> rd_ptr != rd_ptr_prev);
            end: ptrs_depth_gt1
        end: depth_gt1
    endgenerate

//    ast_flush: assert property (flush |=> empty && !full && !udf && !ovf); // Remove flush assume before prooving

    sequence write_not_read;
        wr && ~rd && ~flush;
    endsequence

    sequence wr_data1;
        (wr_data == 4'h1);
    endsequence

    sequence read_not_write;
        rd && ~wr && ~flush;
    endsequence

    sequence idle;
        ~rd && ~wr && ~flush;
    endsequence

 `endif
`endif
endmodule
