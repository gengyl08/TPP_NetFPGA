/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        nf10_tpp.v
 *
 *  Library:
 *        hw/std/pcores/nf10_tpp_v1_00_a
 *
 *  Module:
 *        nf10_tpp
 *
 *  Author:
 *        Yilong Geng
 *
 *  Description:
 *        Hardwire the hardware interfaces to CPU and vice versa
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

module tpp
#(
    parameter C_FAMILY              = "virtex5",
    //Master AXI Stream Data Width
    parameter C_M_AXIS_DATA_WIDTH=256,
    parameter C_S_AXIS_DATA_WIDTH=256,
    parameter C_M_AXIS_TUSER_WIDTH=128,
    parameter C_S_AXIS_TUSER_WIDTH=128,

    parameter TPP_STAGE=0,
    parameter TPP_REG_WRITE_PERMISSION = 8'b11111111,
    parameter TPP_REG0_DEFAULT = 64'd0,
    parameter TPP_REG1_DEFAULT = 64'd0,
    parameter TPP_REG2_DEFAULT = 64'd0,
    parameter TPP_REG3_DEFAULT = 64'd0,
    parameter TPP_REG4_DEFAULT = 64'd0,
    parameter TPP_REG5_DEFAULT = 64'd0,
    parameter TPP_REG6_DEFAULT = 64'd0,
    parameter TPP_REG7_DEFAULT = 64'd0
)
(
    // TPP Ports
    input [63:0] tpp_reg0_in,
    input [63:0] tpp_reg1_in,
    input [63:0] tpp_reg2_in,
    input [63:0] tpp_reg3_in,
    input [63:0] tpp_reg4_in,
    input [63:0] tpp_reg5_in,
    input [63:0] tpp_reg6_in,
    input [63:0] tpp_reg7_in,

    output [63:0] tpp_reg0_out,
    output [63:0] tpp_reg1_out,
    output [63:0] tpp_reg2_out,
    output [63:0] tpp_reg3_out,
    output [63:0] tpp_reg4_out,
    output [63:0] tpp_reg5_out,
    output [63:0] tpp_reg6_out,
    output [63:0] tpp_reg7_out,

    // Global Ports
    input axi_aclk,
    input axi_resetn,

    // Master Stream Ports (interface to data path)
    output reg [C_M_AXIS_DATA_WIDTH - 1:0] m_axis_tdata,
    output reg [((C_M_AXIS_DATA_WIDTH / 8)) - 1:0] m_axis_tstrb,
    output reg [C_M_AXIS_TUSER_WIDTH-1:0] m_axis_tuser,
    output reg m_axis_tvalid,
    input  m_axis_tready,
    output reg m_axis_tlast,

    // Slave Stream Ports (interface to RX queues)
    input [C_S_AXIS_DATA_WIDTH - 1:0] s_axis_tdata,
    input [((C_S_AXIS_DATA_WIDTH / 8)) - 1:0] s_axis_tstrb,
    input [C_S_AXIS_TUSER_WIDTH-1:0] s_axis_tuser,
    input  s_axis_tvalid,
    output s_axis_tready,
    input  s_axis_tlast
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

   // ------------ TPP REGs --------------
   reg [63:0] tpp_reg0, tpp_reg0_next;
   reg [63:0] tpp_reg1, tpp_reg1_next;
   reg [63:0] tpp_reg2, tpp_reg2_next;
   reg [63:0] tpp_reg3, tpp_reg3_next;
   reg [63:0] tpp_reg4, tpp_reg4_next;
   reg [63:0] tpp_reg5, tpp_reg5_next;
   reg [63:0] tpp_reg6, tpp_reg6_next;
   reg [63:0] tpp_reg7, tpp_reg7_next;

   assign tpp_reg0_out = tpp_reg0;
   assign tpp_reg1_out = tpp_reg1;
   assign tpp_reg2_out = tpp_reg2;
   assign tpp_reg3_out = tpp_reg3;
   assign tpp_reg4_out = tpp_reg4;
   assign tpp_reg5_out = tpp_reg5;
   assign tpp_reg6_out = tpp_reg6;
   assign tpp_reg7_out = tpp_reg7;

   always @(posedge axi_aclk) begin
      if(~axi_resetn) begin
	 tpp_reg0 <= TPP_REG0_DEFAULT;
	 tpp_reg1 <= TPP_REG1_DEFAULT;
	 tpp_reg2 <= TPP_REG2_DEFAULT;
	 tpp_reg3 <= TPP_REG3_DEFAULT;
	 tpp_reg4 <= TPP_REG4_DEFAULT;
	 tpp_reg5 <= TPP_REG5_DEFAULT;
	 tpp_reg6 <= TPP_REG6_DEFAULT;
	 tpp_reg7 <= TPP_REG7_DEFAULT;
      end
      else begin
	 if(~TPP_REG_WRITE_PERMISSION[0]) begin
             tpp_reg0 <= tpp_reg0_in;
         end
         else begin
             tpp_reg0 <= tpp_reg0_next;
         end

	 if(~TPP_REG_WRITE_PERMISSION[1]) begin
             tpp_reg1 <= tpp_reg1_in;
         end
         else begin
             tpp_reg1 <= tpp_reg1_next;
         end

	 if(~TPP_REG_WRITE_PERMISSION[2]) begin
             tpp_reg2 <= tpp_reg2_in;
         end
         else begin
             tpp_reg2 <= tpp_reg2_next;
         end

	 if(~TPP_REG_WRITE_PERMISSION[3]) begin
             tpp_reg3 <= tpp_reg3_in;
         end
         else begin
             tpp_reg3 <= tpp_reg3_next;
         end

	 if(~TPP_REG_WRITE_PERMISSION[4]) begin
             tpp_reg4 <= tpp_reg4_in;
         end
         else begin
             tpp_reg4 <= tpp_reg4_next;
         end

	 if(~TPP_REG_WRITE_PERMISSION[5]) begin
             tpp_reg5 <= tpp_reg5_in;
         end
         else begin
             tpp_reg5 <= tpp_reg5_next;
         end

	 if(~TPP_REG_WRITE_PERMISSION[6]) begin
             tpp_reg6 <= tpp_reg6_in;
         end
         else begin
             tpp_reg6 <= tpp_reg6_next;
         end

	 if(~TPP_REG_WRITE_PERMISSION[7]) begin
             tpp_reg7 <= tpp_reg7_in;
         end
         else begin
             tpp_reg7 <= tpp_reg7_next;
         end
      end
   end

   // ------------ Internal Params --------
   localparam MODULE_HEADER = 0;
   localparam HEADER_2      = 1;
   localparam INS           = 2;
   localparam INS_LOAD_REG_HOPMEM = 3;
   localparam INS_LOAD_SMEM_HOPMEM = 4;
   localparam INS_STORE_HOPMEM_REG = 5;
   localparam INS_STORE_HOPMEM_SMEM = 6;
   localparam INS_STOREC_HOPMEM_HOPMEM_REG_0 = 7;
   localparam INS_STOREC_HOPMEM_HOPMEM_REG_1 = 8;
   localparam INS_STOREC_HOPMEM_HOPMEM_REG_2 = 9;
   localparam INS_STOREC_HOPMEM_HOPMEM_SMEM_0 = 11;
   localparam INS_STOREC_HOPMEM_HOPMEM_SMEM_1 = 12;
   localparam INS_STOREC_HOPMEM_HOPMEM_SMEM_2 = 13;
   localparam IN_PACKET     = 14;

   localparam LOAD_REG_HOPMEM = 0;
   localparam LOAD_SMEM_HOPMEM = 1;
   localparam STORE_HOPMEM_REG = 2;
   localparam STORE_HOPMEM_SMEM = 3;
   localparam STOREC_HOPMEM_HOPMEM_REG = 4;
   localparam STOREC_HOPMEM_HOPMEM_SMEM = 5;

   //------------- Wires ------------------
   wire [255:0] tdata_fifo_out;
   wire  [C_M_AXIS_TUSER_WIDTH-1:0] tuser_fifo_out;
   wire [31:0] tstrb_fifo_out;
   wire tlast_fifo_out;

   reg in_fifo_rd_en;

   reg [7:0] tpp_reg_addr, tpp_reg_addr_next;

   reg [3:0] tpp_hopmem_addr_0, tpp_hopmem_addr_0_next;
   reg [3:0] tpp_hopmem_addr_1, tpp_hopmem_addr_1_next;

   reg [63:0] tpp_hopmem_0, tpp_hopmem_0_next, tpp_hopmem_1, tpp_hopmem_1_next;

   reg [3:0] state, state_next;

   // ------------ Modules ----------------

   fallthrough_small_fifo
        #( .WIDTH(C_M_AXIS_DATA_WIDTH+C_M_AXIS_TUSER_WIDTH+C_M_AXIS_DATA_WIDTH/8+1),
           .MAX_DEPTH_BITS(2))
      input_fifo
        (// Outputs
         .dout                           ({tlast_fifo_out, tuser_fifo_out, tstrb_fifo_out, tdata_fifo_out}),
         .full                           (),
         .nearly_full                    (in_fifo_nearly_full),
         .prog_full                      (),
         .empty                          (in_fifo_empty),
         // Inputs
         .din                            ({s_axis_tlast, s_axis_tuser, s_axis_tstrb, s_axis_tdata}),
         .wr_en                          (s_axis_tvalid & ~in_fifo_nearly_full),
         .rd_en                          (in_fifo_rd_en),
         .reset                          (~axi_resetn),
         .clk                            (axi_aclk));

   reg ram_wr_en;
   reg [9:0] ram_addr, ram_addr_reg, ram_addr_next;
   reg [63:0] ram_din;
   wire [63:0] ram_dout;

   tpp_ram ram(axi_aclk, ~axi_resetn, ram_wr_en, ram_addr, ram_din, ram_dout);

   // ------------- Logic ----------------

   assign s_axis_tready = !in_fifo_nearly_full;

   // modify the dst port in tuser
   always @(*) begin
      m_axis_tdata = tdata_fifo_out;
      m_axis_tuser = tuser_fifo_out;
      m_axis_tstrb = tstrb_fifo_out;
      m_axis_tlast = tlast_fifo_out;
      m_axis_tvalid = 0;
      in_fifo_rd_en = 0;
      state_next      = state;
      tpp_reg_addr_next = tpp_reg_addr;
      tpp_reg0_next = tpp_reg0;
      tpp_reg1_next = tpp_reg1;
      tpp_reg2_next = tpp_reg2;
      tpp_reg3_next = tpp_reg3;
      tpp_reg4_next = tpp_reg4;
      tpp_reg5_next = tpp_reg5;
      tpp_reg6_next = tpp_reg6;
      tpp_reg7_next = tpp_reg7;

      ram_wr_en = 0;
      ram_addr = ram_addr_reg;
      ram_din = 0;
      ram_addr_next = ram_addr_reg;

      tpp_hopmem_addr_0_next = tpp_hopmem_addr_0;
      tpp_hopmem_addr_1_next = tpp_hopmem_addr_1;
      tpp_hopmem_0_next = tpp_hopmem_0;
      tpp_hopmem_1_next = tpp_hopmem_1;

      case(state)
	MODULE_HEADER: begin
           if(~in_fifo_empty) begin
              m_axis_tvalid = 1;
              if(m_axis_tready) begin
                 in_fifo_rd_en = 1;
                 if(~m_axis_tlast) begin
                    state_next = HEADER_2;
                 end
                 else begin
                    state_next = MODULE_HEADER;
                 end
              end
           end
	end // case: MODULE_HEADER

        HEADER_2: begin
           if(~in_fifo_empty) begin
              m_axis_tvalid = 1;
              if(m_axis_tready) begin
                 in_fifo_rd_en = 1;
                 if(~m_axis_tlast) begin
                    state_next = INS;
                 end
                 else begin
                    state_next = MODULE_HEADER;
                 end
              end
           end
        end

        INS: begin
           if(~in_fifo_empty) begin
              m_axis_tvalid = 1;
              if(m_axis_tready) begin
                 in_fifo_rd_en = 1;
                 if(~m_axis_tlast) begin
                    case(m_axis_tdata[7+TPP_STAGE*32:TPP_STAGE*32])

                       LOAD_REG_HOPMEM: begin
                          tpp_reg_addr_next = m_axis_tdata[15+TPP_STAGE*32:8+TPP_STAGE*32];
                          state_next = INS_LOAD_REG_HOPMEM;
                       end

                       LOAD_SMEM_HOPMEM: begin
                          ram_addr = m_axis_tdata[17+TPP_STAGE*32:8+TPP_STAGE*32];
                          ram_addr_next = m_axis_tdata[17+TPP_STAGE*32:8+TPP_STAGE*32];
                          state_next = INS_LOAD_SMEM_HOPMEM;
                       end

                       STORE_HOPMEM_REG: begin
                          tpp_reg_addr_next = m_axis_tdata[15+TPP_STAGE*32:8+TPP_STAGE*32];
                          state_next = INS_STORE_HOPMEM_REG;
                       end

                       STORE_HOPMEM_SMEM: begin
                          ram_addr_next = m_axis_tdata[17+TPP_STAGE*32:8+TPP_STAGE*32];
                          state_next = INS_STORE_HOPMEM_SMEM;
                       end

                       STOREC_HOPMEM_HOPMEM_REG: begin
                          tpp_reg_addr_next = m_axis_tdata[23+TPP_STAGE*32:16+TPP_STAGE*32];
                          tpp_hopmem_addr_0_next = m_axis_tdata[11+TPP_STAGE*32:8+TPP_STAGE*32];
                          tpp_hopmem_addr_1_next = m_axis_tdata[15+TPP_STAGE*32:12+TPP_STAGE*32];
                          state_next = INS_STOREC_HOPMEM_HOPMEM_REG_0;
                       end

                       STOREC_HOPMEM_HOPMEM_SMEM: begin
                          ram_addr_next = m_axis_tdata[25+TPP_STAGE*32:16+TPP_STAGE*32];
                          tpp_hopmem_addr_0_next = m_axis_tdata[11+TPP_STAGE*32:8+TPP_STAGE*32];
                          tpp_hopmem_addr_1_next = m_axis_tdata[15+TPP_STAGE*32:12+TPP_STAGE*32];
                          state_next = INS_STOREC_HOPMEM_HOPMEM_SMEM_0;
                       end

                       default: begin
                          state_next = IN_PACKET;
                       end

                    endcase
                 end
                 else begin
                    state_next = MODULE_HEADER;
                 end
              end
           end
        end

        INS_LOAD_REG_HOPMEM: begin

           case(tpp_reg_addr[2:0])
              3'd0: begin
                 m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE] = tpp_reg0;
              end

              3'd1: begin
                 m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE] = tpp_reg1;
              end

              3'd2: begin
                 m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE] = tpp_reg2;
              end

              3'd3: begin
                 m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE] = tpp_reg3;
              end

              3'd4: begin
                 m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE] = tpp_reg4;
              end

              3'd5: begin
                 m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE] = tpp_reg5;
              end

              3'd6: begin
                 m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE] = tpp_reg6;
              end

              3'd7: begin
                 m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE] = tpp_reg7;
              end
           endcase

           if(~in_fifo_empty) begin
              m_axis_tvalid = 1;

              if(m_axis_tready) begin
                 in_fifo_rd_en = 1;
                 if(~m_axis_tlast) begin
                    state_next = IN_PACKET;
                 end
                 else begin
                    state_next = MODULE_HEADER;
                 end
              end
           end
        end

        INS_LOAD_SMEM_HOPMEM: begin
           m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE] = ram_dout;
           if(~in_fifo_empty) begin
              m_axis_tvalid = 1;
              if(m_axis_tready) begin
                 in_fifo_rd_en = 1;
                 if(~m_axis_tlast) begin
                    state_next = IN_PACKET;
                 end
                 else begin
                    state_next = MODULE_HEADER;
                 end
              end
           end
        end

        INS_STORE_HOPMEM_REG: begin
           if(~in_fifo_empty) begin
              m_axis_tvalid = 1;
              case(tpp_reg_addr[2:0])
                 3'd0: begin
                    tpp_reg0_next = m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE];
                 end

                 3'd1: begin
                    tpp_reg1_next = m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE];
                 end

                 3'd2: begin
                    tpp_reg2_next = m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE];
                 end

                 3'd3: begin
                    tpp_reg3_next = m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE];
                 end

                 3'd4: begin
                    tpp_reg4_next = m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE];
                 end

                 3'd5: begin
                    tpp_reg5_next = m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE];
                 end

                 3'd6: begin
                    tpp_reg6_next = m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE];
                 end

                 3'd7: begin
                    tpp_reg7_next = m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE];
                 end
              endcase

              if(m_axis_tready) begin
                 in_fifo_rd_en = 1;
                 if(~m_axis_tlast) begin
                    state_next = IN_PACKET;
                 end
                 else begin
                    state_next = MODULE_HEADER;
                 end
              end
           end
        end

        INS_STORE_HOPMEM_SMEM: begin
           if(~in_fifo_empty) begin
              m_axis_tvalid = 1;
              if(m_axis_tready) begin
                 in_fifo_rd_en = 1;
                 ram_din = m_axis_tdata[63+64*TPP_STAGE:64*TPP_STAGE];
                 ram_wr_en = 1;
                 if(~m_axis_tlast) begin
                    state_next = IN_PACKET;
                 end
                 else begin
                    state_next = MODULE_HEADER;
                 end
              end
           end
        end

        INS_STOREC_HOPMEM_HOPMEM_REG_0: begin

           case(tpp_hopmem_addr_0[1:0])
             2'd0: begin
                tpp_hopmem_0_next = m_axis_tdata[63:0];
             end
        
             2'd1: begin
                tpp_hopmem_0_next = m_axis_tdata[127:64];
             end

             2'd2: begin
                tpp_hopmem_0_next = m_axis_tdata[191:128];
             end

             2'd3: begin
                tpp_hopmem_0_next = m_axis_tdata[255:192];
             end   
           endcase

           case(tpp_hopmem_addr_1[1:0])
             2'd0: begin
                tpp_hopmem_1_next = m_axis_tdata[63:0];
             end
        
             2'd1: begin
                tpp_hopmem_1_next = m_axis_tdata[127:64];
             end

             2'd2: begin
                tpp_hopmem_1_next = m_axis_tdata[191:128];
             end

             2'd3: begin
                tpp_hopmem_1_next = m_axis_tdata[255:192];
             end   
           endcase

           if(~in_fifo_empty) begin
              m_axis_tvalid = 1;

              if(m_axis_tready) begin
                 in_fifo_rd_en = 1;
                 if(~m_axis_tlast) begin
                    state_next = INS_STOREC_HOPMEM_HOPMEM_REG_1;
                 end
                 else begin
                    state_next = INS_STOREC_HOPMEM_HOPMEM_REG_2;
                 end
              end
           end
        end

        INS_STOREC_HOPMEM_HOPMEM_REG_1: begin
           if(tpp_hopmem_0 == tpp_hopmem_1) begin
              case(tpp_reg_addr[2:0])
                 3'd0: begin
                    tpp_reg0_next = tpp_hopmem_1;
                 end

                 3'd1: begin
                    tpp_reg1_next = tpp_hopmem_1;
                 end

                 3'd2: begin
                    tpp_reg2_next = tpp_hopmem_1;
                 end

                 3'd3: begin
                    tpp_reg3_next = tpp_hopmem_1;
                 end

                 3'd4: begin
                    tpp_reg4_next = tpp_hopmem_1;
                 end

                 3'd5: begin
                    tpp_reg5_next = tpp_hopmem_1;
                 end

                 3'd6: begin
                    tpp_reg6_next = tpp_hopmem_1;
                 end

                 3'd7: begin
                    tpp_reg7_next = tpp_hopmem_1;
                 end
              endcase
           end
           if(~in_fifo_empty) begin
              m_axis_tvalid = 1;
              if(m_axis_tready) begin
                 in_fifo_rd_en = 1;
                 if(~m_axis_tlast) begin
                    state_next = IN_PACKET;
                 end
                 else begin
                    state_next = MODULE_HEADER;
                 end
              end
           end
        end

        INS_STOREC_HOPMEM_HOPMEM_REG_2: begin
           if(tpp_hopmem_0 == tpp_hopmem_1) begin
              case(tpp_reg_addr[2:0])
                 3'd0: begin
                    tpp_reg0_next = tpp_hopmem_1;
                 end

                 3'd1: begin
                    tpp_reg1_next = tpp_hopmem_1;
                 end

                 3'd2: begin
                    tpp_reg2_next = tpp_hopmem_1;
                 end

                 3'd3: begin
                    tpp_reg3_next = tpp_hopmem_1;
                 end

                 3'd4: begin
                    tpp_reg4_next = tpp_hopmem_1;
                 end

                 3'd5: begin
                    tpp_reg5_next = tpp_hopmem_1;
                 end

                 3'd6: begin
                    tpp_reg6_next = tpp_hopmem_1;
                 end

                 3'd7: begin
                    tpp_reg7_next = tpp_hopmem_1;
                 end
              endcase
           end
           state_next = MODULE_HEADER;
        end

        INS_STOREC_HOPMEM_HOPMEM_SMEM_0: begin

           case(tpp_hopmem_addr_0[1:0])
             2'd0: begin
                tpp_hopmem_0_next = m_axis_tdata[63:0];
             end
        
             2'd1: begin
                tpp_hopmem_0_next = m_axis_tdata[127:64];
             end

             2'd2: begin
                tpp_hopmem_0_next = m_axis_tdata[191:128];
             end

             2'd3: begin
                tpp_hopmem_0_next = m_axis_tdata[255:192];
             end   
           endcase

           case(tpp_hopmem_addr_1[1:0])
             2'd0: begin
                tpp_hopmem_1_next = m_axis_tdata[63:0];
             end
        
             2'd1: begin
                tpp_hopmem_1_next = m_axis_tdata[127:64];
             end

             2'd2: begin
                tpp_hopmem_1_next = m_axis_tdata[191:128];
             end

             2'd3: begin
                tpp_hopmem_1_next = m_axis_tdata[255:192];
             end   
           endcase

           if(~in_fifo_empty) begin
              m_axis_tvalid = 1;
              if(m_axis_tready) begin
                 in_fifo_rd_en = 1;
                 if(~m_axis_tlast) begin
                    state_next = INS_STOREC_HOPMEM_HOPMEM_SMEM_1;
                 end
                 else begin
                    state_next = INS_STOREC_HOPMEM_HOPMEM_SMEM_2;
                 end
              end
           end
        end

        INS_STOREC_HOPMEM_HOPMEM_SMEM_1: begin
            if(tpp_hopmem_0 == tpp_hopmem_1) begin
                ram_din = tpp_hopmem_1;
                ram_wr_en = 1;
            end

           if(~in_fifo_empty) begin
              m_axis_tvalid = 1;
              if(m_axis_tready) begin
                 in_fifo_rd_en = 1;
                 if(~m_axis_tlast) begin
                    state_next = IN_PACKET;
                 end
                 else begin
                    state_next = MODULE_HEADER;
                 end
              end
           end
        end

        INS_STOREC_HOPMEM_HOPMEM_SMEM_2: begin
            if(tpp_hopmem_0 == tpp_hopmem_1) begin
                ram_din = tpp_hopmem_1;
                ram_wr_en = 1;
            end

            state_next = MODULE_HEADER;
        end

	IN_PACKET: begin
           if(~in_fifo_empty) begin
              m_axis_tvalid = 1;
              if(m_axis_tready) begin
                 in_fifo_rd_en = 1;
                 if(m_axis_tlast) begin
                    state_next = MODULE_HEADER;
                 end
              end
           end
	end
      endcase // case (state)
   end // always @ (*)

   always @(posedge axi_aclk) begin
      if(~axi_resetn) begin
	 state <= MODULE_HEADER;
         tpp_reg_addr <= 0;
         ram_addr_reg <= 0;
         tpp_hopmem_addr_0 <= 0;
         tpp_hopmem_addr_1 <= 0;
         tpp_hopmem_0 <= 0;
         tpp_hopmem_1 <= 0;
      end
      else begin
	 state <= state_next;
         tpp_reg_addr <= tpp_reg_addr_next;
         ram_addr_reg <= ram_addr_next;
         tpp_hopmem_addr_0 <= tpp_hopmem_addr_0_next;
         tpp_hopmem_addr_1 <= tpp_hopmem_addr_1_next;
         tpp_hopmem_0 <= tpp_hopmem_0_next;
         tpp_hopmem_1 <= tpp_hopmem_1_next;
      end
   end

endmodule // output_port_lookup
