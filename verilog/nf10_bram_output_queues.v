/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        nf10_bram_output_queues.v
 *
 *  Library:
 *        hw/std/pcores/nf10_bram_output_queues_v1_00_a
 *
 *  Module:
 *        nf10_bram_output_queues
 *
 *  Author:
 *        James Hongyi Zeng
 *
 *  Description:
 *        BRAM Output queues
 *        Outputs have a parameterizable width
 *
 *  Copyright notice:
 *        Copyright (C) 2010, 2011 The Board of Trustees of The Leland Stanford
 *                                 Junior University
 *
 *  Licence:
 *        This file is part of the NetFPGA 10G development base package.
 *
 *        This file is free code: you can redistribute it and/or modify it under
 *        the terms of the GNU Lesser General Public License version 2.1 as
 *        published by the Free Software Foundation.
 *
 *        This package is distributed in the hope that it will be useful, but
 *        WITHOUT ANY WARRANTY; without even the implied warranty of
 *        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *        Lesser General Public License for more details.
 *
 *        You should have received a copy of the GNU Lesser General Public
 *        License along with the NetFPGA source package.  If not, see
 *        http://www.gnu.org/licenses/.
 *
 */

module nf10_bram_output_queues
#(
    // Master AXI Stream Data Width
    parameter C_M_AXIS_DATA_WIDTH=256,
    parameter C_S_AXIS_DATA_WIDTH=256,
    parameter C_M_AXIS_TUSER_WIDTH=128,
    parameter C_S_AXIS_TUSER_WIDTH=128,
    parameter NUM_QUEUES=5
)
(
    // Part 1: System side signals
    // Global Ports
    input axi_aclk,
    input axi_resetn,

    // Slave Stream Ports (interface to data path)
    input [C_S_AXIS_DATA_WIDTH - 1:0] s_axis_tdata,
    input [((C_S_AXIS_DATA_WIDTH / 8)) - 1:0] s_axis_tstrb,
    input [C_S_AXIS_TUSER_WIDTH-1:0] s_axis_tuser,
    input s_axis_tvalid,
    output reg s_axis_tready,
    input s_axis_tlast,

    // Master Stream Ports (interface to TX queues)
    output [C_M_AXIS_DATA_WIDTH - 1:0] m_axis_tdata_0,
    output [((C_M_AXIS_DATA_WIDTH / 8)) - 1:0] m_axis_tstrb_0,
    output [C_M_AXIS_TUSER_WIDTH-1:0] m_axis_tuser_0,
    output  m_axis_tvalid_0,
    input m_axis_tready_0,
    output  m_axis_tlast_0,

    output [C_M_AXIS_DATA_WIDTH - 1:0] m_axis_tdata_1,
    output [((C_M_AXIS_DATA_WIDTH / 8)) - 1:0] m_axis_tstrb_1,
    output [C_M_AXIS_TUSER_WIDTH-1:0] m_axis_tuser_1,
    output  m_axis_tvalid_1,
    input m_axis_tready_1,
    output  m_axis_tlast_1,

    output [C_M_AXIS_DATA_WIDTH - 1:0] m_axis_tdata_2,
    output [((C_M_AXIS_DATA_WIDTH / 8)) - 1:0] m_axis_tstrb_2,
    output [C_M_AXIS_TUSER_WIDTH-1:0] m_axis_tuser_2,
    output  m_axis_tvalid_2,
    input m_axis_tready_2,
    output  m_axis_tlast_2,

    output [C_M_AXIS_DATA_WIDTH - 1:0] m_axis_tdata_3,
    output [((C_M_AXIS_DATA_WIDTH / 8)) - 1:0] m_axis_tstrb_3,
    output [C_M_AXIS_TUSER_WIDTH-1:0] m_axis_tuser_3,
    output  m_axis_tvalid_3,
    input m_axis_tready_3,
    output  m_axis_tlast_3,

    output [C_M_AXIS_DATA_WIDTH - 1:0] m_axis_tdata_4,
    output [((C_M_AXIS_DATA_WIDTH / 8)) - 1:0] m_axis_tstrb_4,
    output [C_M_AXIS_TUSER_WIDTH-1:0] m_axis_tuser_4,
    output  m_axis_tvalid_4,
    input m_axis_tready_4,
    output  m_axis_tlast_4
);

   function integer log2;
      input integer number;
      begin
         log2=0;
         while(2**log2<number) begin
            log2=log2+1;
         end
      end
   endfunction // log2

   // ------------ Internal Params --------

   localparam NUM_QUEUES_WIDTH = log2(NUM_QUEUES);

   localparam BUFFER_SIZE         = 4096; // Buffer size 4096B
   localparam BUFFER_SIZE_WIDTH   = log2(BUFFER_SIZE/(C_M_AXIS_DATA_WIDTH/8));

   localparam MAX_PACKET_SIZE = 1600;
   localparam BUFFER_THRESHOLD = (BUFFER_SIZE-MAX_PACKET_SIZE)/(C_M_AXIS_DATA_WIDTH/8);

   localparam NUM_STATES = 3;
   localparam IDLE = 0;
   localparam WR_PKT = 1;
   localparam DROP = 2;

   localparam NUM_METADATA_STATES = 2;
   localparam WAIT_HEADER = 0;
   localparam WAIT_EOP = 1;

   // ------------- Regs/ wires -----------

   reg [NUM_QUEUES-1:0]                nearly_full;
   wire [NUM_QUEUES-1:0]               nearly_full_fifo;
   wire [NUM_QUEUES-1:0]               empty;

   reg [NUM_QUEUES-1:0]                metadata_nearly_full;
   wire [NUM_QUEUES-1:0]               metadata_nearly_full_fifo;
   wire [NUM_QUEUES-1:0]               metadata_empty;

   wire [C_M_AXIS_TUSER_WIDTH-1:0]             fifo_out_tuser[NUM_QUEUES-1:0];
   wire [C_M_AXIS_DATA_WIDTH-1:0]        fifo_out_tdata[NUM_QUEUES-1:0];
   wire [((C_M_AXIS_DATA_WIDTH/8))-1:0]  fifo_out_tstrb[NUM_QUEUES-1:0];
   wire [NUM_QUEUES-1:0] 	           fifo_out_tlast;

   wire [C_M_AXIS_TUSER_WIDTH-1:0]             m_axis_tuser[NUM_QUEUES-1:0];
   wire [C_M_AXIS_DATA_WIDTH-1:0]        m_axis_tdata[NUM_QUEUES-1:0];
   wire [((C_M_AXIS_DATA_WIDTH/8))-1:0]  m_axis_tstrb[NUM_QUEUES-1:0];
   wire [NUM_QUEUES-1:0] 	           m_axis_tlast;
   wire [NUM_QUEUES-1:0] 	           m_axis_tvalid;
   wire [NUM_QUEUES-1:0] 	           m_axis_tready;
   wire [NUM_QUEUES-1:0] 	           tpp_s_axis_tready;

   wire [NUM_QUEUES-1:0]               rd_en;
   reg [NUM_QUEUES-1:0]                wr_en;

   reg [NUM_QUEUES-1:0]                metadata_rd_en;
   reg [NUM_QUEUES-1:0]                metadata_wr_en;

   reg [NUM_QUEUES-1:0]          cur_queue;
   reg [NUM_QUEUES-1:0]          cur_queue_next;
   wire [NUM_QUEUES-1:0]         oq;

   reg [NUM_STATES-1:0]                state;
   reg [NUM_STATES-1:0]                state_next;

   reg [NUM_METADATA_STATES-1:0]       metadata_state[NUM_QUEUES-1:0];
   reg [NUM_METADATA_STATES-1:0]       metadata_state_next[NUM_QUEUES-1:0];

   reg								   first_word, first_word_next;


   // ---------------TPP----------------
   reg [63:0] queue_size[NUM_QUEUES-1:0];
   reg [63:0] queue_size_next[NUM_QUEUES-1:0];
   reg [63:0] queue_util_counter[NUM_QUEUES-1:0];
   reg [63:0] queue_util_counter_next[NUM_QUEUES-1:0];
   reg [63:0] queue_util[NUM_QUEUES-1:0];
   reg [63:0] queue_util_next[NUM_QUEUES-1:0];
   reg [63:0] timer[NUM_QUEUES-1:0];
   wire [63:0] timer_max[NUM_QUEUES-1:0];


   // ------------ Modules -------------

   generate
   genvar i;
   for(i=0; i<NUM_QUEUES; i=i+1) begin: output_queues
      fallthrough_small_fifo
        #( .WIDTH(C_M_AXIS_DATA_WIDTH+C_M_AXIS_DATA_WIDTH/8+1),
           .MAX_DEPTH_BITS(BUFFER_SIZE_WIDTH),
           .PROG_FULL_THRESHOLD(BUFFER_THRESHOLD))
      output_fifo
        (// Outputs
         .dout                           ({fifo_out_tlast[i], fifo_out_tstrb[i], fifo_out_tdata[i]}),
         .full                           (),
         .nearly_full                    (),
	 	 .prog_full                      (nearly_full_fifo[i]),
         .empty                          (empty[i]),
         // Inputs
         .din                            ({s_axis_tlast, s_axis_tstrb, s_axis_tdata}),
         .wr_en                          (wr_en[i]),
         .rd_en                          (rd_en[i]),
         .reset                          (~axi_resetn),
         .clk                            (axi_aclk));

      fallthrough_small_fifo
        #( .WIDTH(C_M_AXIS_TUSER_WIDTH),
           .MAX_DEPTH_BITS(2))
      metadata_fifo
        (// Outputs
         .dout                           (fifo_out_tuser[i]),
         .full                           (),
         .nearly_full                    (metadata_nearly_full_fifo[i]),
	 	 .prog_full                      (),
         .empty                          (metadata_empty[i]),
         // Inputs
         .din                            (s_axis_tuser),
         .wr_en                          (metadata_wr_en[i]),
         .rd_en                          (metadata_rd_en[i]),
         .reset                          (~axi_resetn),
         .clk                            (axi_aclk));

      tpp
        #(.TPP_REG_WRITE_PERMISSION(8'b00000100),
          .TPP_REG2_DEFAULT(64'd160000))
      tpp_processor
        (.tpp_reg0_in(queue_size[i]),
         .tpp_reg1_in(queue_util[i]),
         .tpp_reg3_in(timer[i]),
         .tpp_reg2_out(timer_max[i]),

         .axi_aclk(axi_aclk),
         .axi_resetn(axi_resetn),

         .m_axis_tdata(m_axis_tdata[i]),
         .m_axis_tstrb(m_axis_tstrb[i]),
         .m_axis_tuser(m_axis_tuser[i]),
         .m_axis_tvalid(m_axis_tvalid[i]),
         .m_axis_tready(m_axis_tready[i]),
         .m_axis_tlast(m_axis_tlast[i]),

         .s_axis_tdata(fifo_out_tdata[i]),
         .s_axis_tstrb(fifo_out_tstrb[i]),
         .s_axis_tuser(fifo_out_tuser[i]),
         .s_axis_tvalid(~empty[i]),
         .s_axis_tready(tpp_s_axis_tready[i]),
         .s_axis_tlast(fifo_out_tlast[i]));

   always @(metadata_state[i], rd_en[i], wr_en[i], fifo_out_tlast[i], timer[i], timer_max[i]) begin
        metadata_rd_en[i] = 1'b0;
        metadata_state_next[i] = metadata_state[i];
      	case(metadata_state[i])
      		WAIT_HEADER: begin
      			if(rd_en[i]) begin
      				metadata_state_next[i] = WAIT_EOP;
      				metadata_rd_en[i] = 1'b1;
      			end
      		end
      		WAIT_EOP: begin
      			if(rd_en[i] & fifo_out_tlast[i]) begin
      				metadata_state_next[i] = WAIT_HEADER;
      			end
      		end
        endcase

        queue_size_next[i] = queue_size[i];
        if(wr_en[i] & ~rd_en[i]) begin
            queue_size_next[i] = queue_size[i] + 1;
        end
        else if(~wr_en[i] & rd_en[i]) begin
            queue_size_next[i] = queue_size[i] - 1;
        end

        queue_util_counter_next[i] = queue_util_counter[i];
        queue_util_next[i] = queue_util[i];
        if(timer[i] < timer_max[i]) begin
            if(rd_en[i]) begin
                queue_util_counter_next[i] = queue_util_counter[i] + 1;
            end
        end
        else begin
            queue_util_counter_next[i] = 0;
            queue_util_next[i] = queue_util_counter[i];
        end

      end

      always @(posedge axi_aclk) begin
      	if(~axi_resetn) begin
         	metadata_state[i] <= WAIT_HEADER;
                queue_size[i] <= 0;
                queue_util_counter[i] <= 0;
                queue_util[i] <= 0;
                timer[i] <= 0;
      	end
      	else begin
         	metadata_state[i] <= metadata_state_next[i];
                queue_size[i] <= queue_size_next[i];
                queue_util_counter[i] <= queue_util_counter_next[i];
                queue_util[i] <= queue_util_next[i];
                if(timer[i] >= timer_max[i]) begin
                    timer[i] <= 0;
                end
                else begin
                    timer[i] <= timer[i] + 1;
                end
      	end
      end
   end
   endgenerate

   // Per NetFPGA-10G AXI Spec
   localparam DST_POS = 24;
   assign oq = s_axis_tuser[DST_POS] |
   			   (s_axis_tuser[DST_POS + 2] << 1) |
   			   (s_axis_tuser[DST_POS + 4] << 2) |
   			   (s_axis_tuser[DST_POS + 6] << 3) |
   			   ((s_axis_tuser[DST_POS + 1] | s_axis_tuser[DST_POS + 3] | s_axis_tuser[DST_POS + 5] | s_axis_tuser[DST_POS + 7]) << 4);

   always @(*) begin
      state_next     = state;
      cur_queue_next = cur_queue;
      wr_en          = 0;
      metadata_wr_en = 0;
      s_axis_tready  = 0;
      first_word_next = first_word;

      case(state)

        /* cycle between input queues until one is not empty */
        IDLE: begin
           cur_queue_next = oq;
           if(s_axis_tvalid) begin
              if(~|((nearly_full | metadata_nearly_full) & oq)) begin // All interesting oqs are NOT _nearly_ full (able to fit in the maximum pacekt).
                  state_next = WR_PKT;
                  first_word_next = 1'b1;
              end
              else begin
              	  state_next = DROP;
              end
           end
        end

        /* wait until eop */
        WR_PKT: begin
           s_axis_tready = 1;
           if(s_axis_tvalid) begin
           		first_word_next = 1'b0;
				wr_en = cur_queue;
				if(first_word) begin
					metadata_wr_en = cur_queue;
				end
				if(s_axis_tlast) begin
					state_next = IDLE;
				end
           end
        end // case: WR_PKT

        DROP: begin
           s_axis_tready = 1;
           if(s_axis_tvalid & s_axis_tlast) begin
           	  state_next = IDLE;
           end
        end

      endcase // case(state)
   end // always @ (*)



   always @(posedge axi_aclk) begin
      if(~axi_resetn) begin
         state <= IDLE;
         cur_queue <= 0;
         first_word <= 0;
      end
      else begin
         state <= state_next;
         cur_queue <= cur_queue_next;
         first_word <= first_word_next;
      end

      nearly_full <= nearly_full_fifo;
      metadata_nearly_full <= metadata_nearly_full_fifo;
   end


   assign m_axis_tdata_0	 = m_axis_tdata[0];
   assign m_axis_tstrb_0	 = m_axis_tstrb[0];
   assign m_axis_tuser_0	 = m_axis_tuser[0];
   assign m_axis_tlast_0	 = m_axis_tlast[0];
   assign m_axis_tvalid_0	 = m_axis_tvalid[0];
   assign rd_en[0]		 = tpp_s_axis_tready[0] & ~empty[0];
   assign m_axis_tready[0]       = m_axis_tready_0;

   assign m_axis_tdata_1	 = m_axis_tdata[1];
   assign m_axis_tstrb_1	 = m_axis_tstrb[1];
   assign m_axis_tuser_1	 = m_axis_tuser[1];
   assign m_axis_tlast_1	 = m_axis_tlast[1];
   assign m_axis_tvalid_1	 = m_axis_tvalid[1];
   assign rd_en[1]		 = tpp_s_axis_tready[1] & ~empty[1];
   assign m_axis_tready[1]       = m_axis_tready_1;

   assign m_axis_tdata_2	 = m_axis_tdata[2];
   assign m_axis_tstrb_2	 = m_axis_tstrb[2];
   assign m_axis_tuser_2	 = m_axis_tuser[2];
   assign m_axis_tlast_2	 = m_axis_tlast[2];
   assign m_axis_tvalid_2	 = m_axis_tvalid[2];
   assign rd_en[2]		 = tpp_s_axis_tready[2] & ~empty[2];
   assign m_axis_tready[2]       = m_axis_tready_2;

   assign m_axis_tdata_3	 = m_axis_tdata[3];
   assign m_axis_tstrb_3	 = m_axis_tstrb[3];
   assign m_axis_tuser_3	 = m_axis_tuser[3];
   assign m_axis_tlast_3	 = m_axis_tlast[3];
   assign m_axis_tvalid_3	 = m_axis_tvalid[3];
   assign rd_en[3]		 = tpp_s_axis_tready[3] & ~empty[3];
   assign m_axis_tready[3]       = m_axis_tready_3;

   assign m_axis_tdata_4	 = m_axis_tdata[4];
   assign m_axis_tstrb_4	 = m_axis_tstrb[4];
   assign m_axis_tuser_4	 = m_axis_tuser[4];
   assign m_axis_tlast_4	 = m_axis_tlast[4];
   assign m_axis_tvalid_4	 = m_axis_tvalid[4];
   assign rd_en[4]		 = tpp_s_axis_tready[4] & ~empty[4];
   assign m_axis_tready[4]       = m_axis_tready_4;



endmodule
