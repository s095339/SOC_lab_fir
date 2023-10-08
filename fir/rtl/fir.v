/***********************************************
File: fir.v
Author: 張耀明
description: SOC design HW3
Implement a fir filter using only 1 multiplier
and 1 adder.
***********************************************/
`timescale 1ns / 1ps
module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    //axilite interface==============================
    //write(input)--
    output  wire                     awready,
    output  wire                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    //read(output)---
    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,
    //stream slave (input data)=========================
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 
    //stream master (output data)=======================
    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

/*
Overall system design:
1. Only one multiplier and adder can be used, so if takes about 11~13(?) clk cycles to generated one output data,
2. Read and store all tap into bram. Once input data received, read all taps in the fir and store them in a tap_buffer 
   to prevent additional SRAM access.
3. After takeing about 11~13(?) clk cycles to generated one output data, axis_write outputs it. the calculation process don't need
   to wait for outputs process to finish.
*/

// =====FSM design=========== //
localparam STAT_IDLE = 3'd0;
localparam STAT_STORE_1_INPUT = 3'd1;
localparam STAT_CAL = 3'd2;
localparam STAT_FINISH = 3'd3;
reg [3-1:0]state, next_state;
// =====Axilite ctrl========= //
//fsm of axilite
localparam LITE_idle = 3'd0;
// for axilite write
localparam LITE_wfinish = 3'd1;
// for axilite read
localparam LITE_arready = 3'd2;
localparam LITE_rreq = 3'd3;
localparam LITE_read = 3'd4;
reg [3-1:0] lite_state, next_lite_state;
//axilite read module
reg arready_reg, rvalid_reg;
reg [(pADDR_WIDTH-1):0] araddr_buf;
//axilite write module
reg awready_reg,wready_reg;
reg [(pADDR_WIDTH-1):0] awaddr_buf;
reg [(pDATA_WIDTH-1):0] wdata_buf;
//axilite to config module
reg [1:0] axilite_req; //(req)
reg [(pADDR_WIDTH-1):0] config_addr;
reg [(pDATA_WIDTH-1):0] w_data, r_data;

// =====Axis-read ctrl======= //
// =====Axis-write ctrl====== //
// =====memory ctrl========== //
// =====calculation unit===== //

/*
あなたの声(ap_start)は、道しるべ。
何もない空に、私わ眠るよ
*/



//******************************//
// FSM                          //
//******************************//
/*
FSM design:
state
0.idle  : (1)wait for coefficients(wait for axilite arvalid), if arvalid=1 go to state 1
          (2)wait for ap_start. If ap_start is set, go to state 1.
1.store_1_input: wait for 1 data input from axis. If 1 input is received, go to state 2.
2.calculation: Do the fir calculation and output 1 data. if tlast==1 go to state 3. else go to state 1.
3.finish: send axilite signal (ap_done, ap_idle) to testbench.
*/


//******************************//
// AxiLite Controller           //
//******************************//


//=====lite fsm=====
always@*
    case(lite_state)
        LITE_idle:
            if(arvalid)
                next_lite_state = LITE_arready;
            else if(wready_reg && awready_reg) 
                next_lite_state = LITE_wfinish;
            else
                next_lite_state = LITE_idle;
        LITE_wfinish:// by the time, axilite has already received awaddr and wdata 
            next_lite_state = LITE_idle;
        LITE_arready:
            if(arready && arvalid)
                next_lite_state = LITE_rreq;
            else
                next_lite_state = LITE_arready;
        LITE_rreq:
            if(rready)
                next_lite_state = LITE_read;
            else
                next_lite_state = LITE_rreq;
        LITE_read:
            next_lite_state = LITE_idle;
        default:
        next_lite_state = LITE_idle;
    endcase
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        lite_state <= LITE_idle;
    else
        lite_state <= next_lite_state;
//===axilite_to_config===
always@* begin
    case(lite_state)
        LITE_wfinish:
            config_addr = awaddr_buf; // for write
        //
        //  config_addr = araddr_buf;

        default:
            config_addr = 12'd1; // addr not being used
    endcase
end
always@*  begin
    case(lite_state)
        LITE_wfinish:
            axilite_req = 2'd1; //write
        LITE_rreq:
            axilite_req = 2'd2;
        default:
            axilite_req = 2'd0; // no operation
    endcase
end

always@*  begin
    case(lite_state)
        LITE_wfinish:
            w_data = wdata_buf;
        default:
            w_data = {pDATA_WIDTH{1'b0}};
    endcase
end
//===lite_write=====

assign awready = (lite_state == LITE_idle)? awready_reg : 1'b0;
assign wready =  (lite_state == LITE_idle)? wready_reg : 1'b0;
//hand shake block
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        awready_reg <= 1'b0;
    else
        case(lite_state)
            LITE_idle:
                if(awvalid)
                    awready_reg <= 1'b1;
                else
                    awready_reg <= awready_reg;
            default:
                awready_reg <= 1'b0;
        endcase
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        wready_reg <=1'b0;
    else
        case(lite_state)
            LITE_idle:
                if(wvalid)
                    wready_reg <= 1'b1;
                else
                    wready_reg <= wready_reg;
            default:
                wready_reg <= 1'b0;
        endcase
//data_addr block
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        awaddr_buf <= {pADDR_WIDTH{1'b0}};
    else
        case(lite_state)
            LITE_idle:
                if(awready)
                    awaddr_buf <= awaddr;
                else
                    awaddr_buf <= awaddr_buf;
            LITE_wfinish:
                // clean the buffer
                awaddr_buf <= {pADDR_WIDTH{1'b0}};
            default:
                awaddr_buf <= {pADDR_WIDTH{1'b0}};
        endcase
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        wdata_buf <= {pDATA_WIDTH{1'b0}};
    else
        case(lite_state)
            LITE_idle:
                if(wready)
                    wdata_buf <= wdata;
                else
                    wdata_buf <= wdata_buf;
            LITE_wfinish:
                // clean the buffer
                wdata_buf <= {pDATA_WIDTH{1'b0}};
            default:
                wdata_buf <= {pDATA_WIDTH{1'b0}};
        endcase



//===lite_read======
//wiring

assign arready = arready_reg;
assign rvalid = rvalid_reg;

// handshake block
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        arready_reg <= 1'b0;
    else
        case(lite_state)
            LITE_idle:
                if(arvalid)
                    arready_reg <= 1'b1;
                else
                    arready_reg <= 1'b0;
            default:
                awready_reg <= 1'b0;
        endcase
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        rvalid_reg <= 1'b0;
    else
        case(lite_state)
            LITE_read:
                rvalid_reg<=1'b1;
            default:
                rvalid_reg<=1'b0;
        endcase
// data_addr block
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        araddr_buf <= {pADDR_WIDTH{1'b0}};
    else
        case(lite_state)
            LITE_arready:
                if(arready)
                    araddr_buf <= araddr;
                else
                    araddr_buf <= araddr_buf;
            LITE_rreq:
                araddr_buf <= araddr_buf;
            LITE_read:
                araddr_buf <= {pADDR_WIDTH{1'b0}};
            default:
                araddr_buf <= {pADDR_WIDTH{1'b0}};
        endcase

assign rdata = (lite_state == LITE_read)? 
                        r_data: {pDATA_WIDTH{1'b0}};

//******************************//
// FIR Dataflow                 //
//******************************//
/*
implementation of single fir output:
--------------------------------------------------------------------------------------------------------
clk        :__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__
ss_tdata_  :|data |____________________________________________________________________|data |___
tdata_in   :__|data|_____________________________________________________________________________
mem_cnt    :|0|1    |2    |3    |4    |5    |6    |7    |8   |9    |10    |0
tap_in     :__|tap0 |tap1 |tap2 |tap3 |tap4 |tap5 |tap6 |tap7|tap8 |tap9  |tap10|__________________
data_in    :__|dat0 |dat1 |dat2 |dat3 |dat4 |dat5 |dat6 |dat7|dat8 |dat9  |dat10|__________________
WE         :__/▔▔\_______________________________________________________________________________
outvalid   :______________________________________________________________/▔▔\___________________
---------------------------------------------------------------------------------------------------------
*/


endmodule