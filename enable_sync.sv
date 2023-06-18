module enable_sync #(
	WIDTH = 8
	)(
	// Source Channel
	input  logic src_clk,
	input  logic src_rst_n,
	input  logic [WIDTH-1:0] src_data,
	// Destination Channel
	input  logic dst_clk,
	input  logic dst_rst_n,
	output logic [WIDTH-1:0]dst_data	
);
	localparam SYNC_STAGES = 2;

// Internal Nets and Wires
logic [SYNC_STAGES-1:0] src2dst_en; // Sync signals
logic [SYNC_STAGES-1:0] dst2src_ack; // Sync signals

logic [WIDTH-1:0] src_data_r;
logic [WIDTH-1:0] src_en;

logic [WIDTH-1:0] dst_ack;

// Sync Signals
always_ff @(posedge src_clk or negedge src_rst_n) begin : src_sync
	if(~src_rst_n) begin
		dst2src_ack[0] <= 1'b0;
	end else begin
		 dst2src_ack[0] <= dst_ack;
	end
	for(int i = 1; i < SYNC_STAGES; i = i+1)
		dst2src_ack[i] <= dst2src_ack[i-1];
end

always_ff @(posedge dst_clk or negedge dst_rst_n) begin : dst_sync
	if(~dst_rst_n) begin
		src2dst_ack[0] <= 1'b0;
	end else begin
		 src2dst_ack[0] <= src_en;
	end
	for(int i = 1; i < SYNC_STAGES; i = i+1)
		src2dst_ack[i] <= src2dst_ack[i-1];
end

// Source
always_ff @(posedge src_clk or negedge src_rst_n) begin : src_capture
	if(~src_rst_n) begin
		src_data_r <= '0; 
		src_en     <= 1'b0;
	end else begin
		if (dst2src_ack[SYNC_STAGES-1]) begin
			src_data_r <= src_data; 
			src_en     <= 1'b1;
		end
		else begin
			src_data_r <= src_data_r; 
			src_en     <= 1'b0;
		end
	end
end

// Destination
always_ff @(posedge dst_clk or negedge dst_rst_n) begin : dst_capture
	if(~dst_rst_n) begin
		dst_data <= '0;
		dst_ack  <= 1'b1;
	end else begin
		if (src2dst_en[SYNC_STAGES-1]) begin
			dst_data <= src_data_r;
			dst_ack  <= 1'b1;
		end
		else begin
			dst_data <= dst_data;
			dst_ack  <= 1'b0;
		end
	end
end
endmodule : enable_sync