///////////////////////////////////////////////////////////////////////////
// Copyright All Rights Reserved.
//
/// Author:        Name : Abdullah Iftikhar     
///                Email: abdullah.iftikhar@dreambigsemi.com
/// Date Created:  17th June 2021
/// Description:   async FIFO
/////////////////////////////////////////////////////////////////////////////////
module async_fifo #(
	parameter WIDTH = 8,
	parameter DEPTH = 8,
	)(
	// FIFO Wirte
	input logic 			wr_clk,
	input logic 			wr_rst_n,
	input logic 			wr,
	input logic [WIDTH-1:0] wr_data,
	// FIFO Read
	input logic 			rd_clk,
	input logic 			rd_rst_n,
	input logic 			rd,
	output logic [WIDTH-1:0]rd_data,
	// FIFO Status
	output logic 			full,
	output logic 			empty,
	output logic 			tfull,
	output logic 			tempty	
);
localparam SYNC_STAGES = 2;

// Internal Nets and Wires
logic [DEPTH-1:0][WIDTH-1:0]mem_array;
logic [$bits(DEPTH):0]wr_ptr; // Addr width + 1 bit to keep track of roll over
logic [$bits(DEPTH):0]rd_ptr; // Addr width + 1 bit to keep track of roll over

logic [SYNC_STAGES-1:0][$bits(DEPTH):0]rd_to_wr;
logic [SYNC_STAGES-1:0][$bits(DEPTH):0]wr_to_rd;

// wr_ptr update logic
always_ff @(posedge wr_clk or negedge wr_rst_n) begin : wr_ptr
	if(~wr_rst_n) begin
		wr_ptr <= 0;
	end else begin
		if (!full & wr)
			wr_ptr <= wr_ptr + 1;
		else
			wr_ptr <= wr_ptr;
	end
end

// rd_ptr update logic
always_ff @(posedge rd_clk or negedge rd_rst_n) begin : rd_ptr
	if(~rd_rst_n) begin
		rd_ptr <= 0;
	end else begin
		if (!empty & rd)
			rd_ptr <= rd_ptr + 1;
		else
			rd_ptr <= rd_ptr;
	end
end

// Full and Empty Siganls
// TODO: Sync wr_ptr with rd_ptr
assign full  = (wr_ptr[$bits(DEPTH)-1:0] == rd_ptr[$bits(DEPTH)-1:0]) && (wr_ptr[$bits(DEPTH)] != rd_ptr[$bits(DEPTH)]);
assign empty = wr_ptr == rd_ptr;

// Memory Array Update
always_ff @(posedge wr_clk or negedge wr_rst_n) begin : mem_array
	if(~wr_rst_n) begin
		mem_array <= '0;
	end else begin
		if (wr & !full)
			mem_array[wr_ptr[$bits(DEPTH)-1:0]] <= wr_data;
		else
			mem_array <= mem_array;
	end
end

// Read from Memory Array
assign rd_data = mem_array[rd_ptr[$bits(DEPTH)-1:0]];
endmodule : async_fifo