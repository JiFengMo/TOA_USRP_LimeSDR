`timescale 1ns / 1ps

module ssb_toa_axis_stub #(
  parameter integer SAMPLE_WIDTH = 32,
  parameter integer RESULT_WIDTH = 128
) (
  input  wire                     aclk,
  input  wire                     aresetn,

  input  wire [SAMPLE_WIDTH-1:0]  s_axis_iq_tdata,
  input  wire                     s_axis_iq_tvalid,
  output wire                     s_axis_iq_tready,
  input  wire                     s_axis_iq_tlast,

  output reg  [RESULT_WIDTH-1:0]  m_axis_result_tdata,
  output reg                      m_axis_result_tvalid,
  input  wire                     m_axis_result_tready,
  output reg                      m_axis_result_tlast
);

  localparam [15:0] STATUS_VALID = 16'h0001;

  reg [31:0] sample_count;
  reg [31:0] peak_power;
  reg [31:0] peak_index;

  wire signed [15:0] sample_i = s_axis_iq_tdata[15:0];
  wire signed [15:0] sample_q = s_axis_iq_tdata[31:16];
  wire [31:0] sample_power =
      (sample_i * sample_i) + (sample_q * sample_q);

  assign s_axis_iq_tready = !m_axis_result_tvalid || m_axis_result_tready;

  always @(posedge aclk) begin
    if (!aresetn) begin
      sample_count <= 32'd0;
      peak_power <= 32'd0;
      peak_index <= 32'd0;
      m_axis_result_tdata <= {RESULT_WIDTH{1'b0}};
      m_axis_result_tvalid <= 1'b0;
      m_axis_result_tlast <= 1'b0;
    end else begin
      if (m_axis_result_tvalid && m_axis_result_tready) begin
        m_axis_result_tvalid <= 1'b0;
        m_axis_result_tlast <= 1'b0;
      end

      if (s_axis_iq_tvalid && s_axis_iq_tready) begin
        if (sample_power > peak_power) begin
          peak_power <= sample_power;
          peak_index <= sample_count;
        end

        if (s_axis_iq_tlast) begin
          m_axis_result_tdata <= {
              16'd0,           // reserved
              STATUS_VALID,    // status
              32'd0,           // fractional offset Q16.16 placeholder
              peak_power,      // peak metric placeholder
              peak_index       // coarse sample index placeholder
          };
          m_axis_result_tvalid <= 1'b1;
          m_axis_result_tlast <= 1'b1;
          sample_count <= 32'd0;
          peak_power <= 32'd0;
          peak_index <= 32'd0;
        end else begin
          sample_count <= sample_count + 32'd1;
        end
      end
    end
  end

endmodule
