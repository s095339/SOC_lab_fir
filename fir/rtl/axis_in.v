`timescale 1ns / 1ps
module axis_in
#(  
    parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)(
    // testbench <-> axis_in
    input wire tvalid,
    input wire [(pDATA_WIDTH-1):0]tdata,
    input wire tlast,
    output wire tready,

    // axis_in <-> fir_dataflow
    output reg[(pDATA_WIDTH-1):0]strm_data,
    output wire strm_valid,
    input wire fir_ready,

    //signal
    output reg axis_finish, 
    input wire ap_start,
    input wire outfinish,
    //clk rst
    input wire clk,
    input wire rst_n
);

//=====fsm=====//
localparam STRM_IDLE = 3'd0;
localparam STRM_GET_FIRST_INPUT = 3'd1;
localparam STRM_WORK = 3'd2;
localparam STRM_LAST = 3'd3;
reg [2:0]state, next_state;

//=====handshake=====//
//input handshake
reg tready_reg;
//output handshake
reg strm_valid_reg,strm_valid_reg_next;


//***************//
//fsm            //
//***************//
always@*
    case(state)
        STRM_IDLE:
            if(ap_start)
                next_state = STRM_WORK;
            else
                next_state = STRM_IDLE;
        STRM_WORK:
            if(tready && tlast)
                next_state = STRM_IDLE;
            else
                next_state = STRM_WORK;
        STRM_LAST:
            if(fir_ready) 
                next_state = STRM_IDLE;
            else 
                next_state = STRM_LAST;
        default:
            next_state = STRM_IDLE;
    endcase

always@(posedge clk or negedge rst_n)
    if(~rst_n)
        state <= STRM_IDLE;
    else
        state <= next_state;
//***************//
//handshake      //
//***************//
//=====input=====
assign tready = tready_reg;
always@*
    case(state)
        STRM_IDLE:
            tready_reg = ap_start;
        STRM_WORK:
            tready_reg = fir_ready & outfinish; // & strm_valid;
        default:
            tready_reg = 1'b0;
    endcase
//=====output=====
always@(posedge clk or negedge rst_n)
    if(~rst_n)
        strm_data<={pDATA_WIDTH{1'b0}};
    else
        case(state)
            STRM_IDLE:
                if(tready & tvalid)
                    strm_data<=tdata;
                else
                    strm_data<={pDATA_WIDTH{1'b0}};
            STRM_WORK:
                if(tready & tvalid)
                    strm_data<=tdata;
                else
                    strm_data<={pDATA_WIDTH{1'b0}};
            default:
                strm_data<={pDATA_WIDTH{1'b0}};
        endcase

assign strm_valid = strm_valid_reg;
always@*
    case(state)
        STRM_IDLE:
            strm_valid_reg_next = tready;
        STRM_WORK:
            strm_valid_reg_next = tready & tvalid ;//fir_ready;
        STRM_LAST:
            strm_valid_reg_next = tready & tvalid ;//fir_ready;
        default:
            strm_valid_reg_next = 1'b0;
    endcase
always@(posedge clk or negedge rst_n)
    if(~rst_n )
        strm_valid_reg<=1'b0;
    else
        strm_valid_reg<=strm_valid_reg_next;


//=====signal=====


always@(posedge clk or negedge rst_n)
    if(~rst_n)
        axis_finish <= 1'b0;
    else
        if(tready)
            axis_finish <= tlast; 
        else
            axis_finish <= 1'b0;


endmodule