//This module encapsulates all Analogizer adapter SNAC controllers
// Original work by @RndMnkIII. 
// Date: 01/2024 
// Release: 1.0

// *** Analogizer R.1 adapter ***
//SNAC mappings:
//USB 3 Type A connector   ______________________________________________________________________________________                             
//PIN_NUMBER:             /           1      2       3       4       5       6       7       8       9           \
//PIN_NAME:              |             VBUS   D-      D+      GND     RX-     RX+     GND_D   TX-     TX+         |
//FUNCTION:              |             +5V    OUT1    OUT2    GND     IO3     IN4     IO5     IO6     IN7         |
//                       |  A                   ^       ^              ^       |       ^       ^       |          |
//                       |  N                   |       |              |       |    +--|-------|-------+          |
//                       |  A       +-----------|-------|--------------|-------+    |  |       +----------------+ |                              
//                       |  L       |         +-|-------|--------------|------------+  +--------+               | |            
//                       |  O       |         | |       |              |      +------------+    |               | |
//                       |  G       |         | |       |              |   +->| B        B |----|------+        | |
//                       |  I       |         | |       |              +---|->| CONF. SW.  |<---+      |        | |
//                       |  Z       |         | |   +---|-------<----------|<-| A        A |<----+     |        | |
//                       |  E       |         | |   |   |                  |  +------------+     |     |        | |
//                       |  R       |         | |   |   |             +-->-+------>--------->----+     |        | |         
//                       |         I|        I| |   |   +-------+     |                                |        | |
//                       |  R      N|        N| +---------+     |    B|A     B OUT                     |        | |
//                       |  1      4|   I    7|  IO3| OUT1| OUT2|  IO3|IO5  +-----------------------------------+ |
//                       |          |   O   IN|    A|     |O    |O   O|O    |A IN                      |          |  
//                       |         I|   5+----|-----|-----|U----|U---U|U ---|--------------------------+          |  
//                       |      ___N|___B|IN__|___IN|_____|T____|T___T|T____|_____                                |  
//POCKET                 |     /    V    V    V     V     ^     ^     ^     V     \                               |    
//CARTRIDGE PIN #:        \___|     2    3    4     5 ... 28    29    30    31     |_____________________________/
//                             \____|____|____|_____|_____|_____|_____|_____|_____/
//Pocket Pin Name:                  |    |    |     |     |     |     |     |
//cart_tran_bank0[7] ---------------+    |    |     |     |     |     |     | cart_tran_bank0_dir=1'b0; //input
//cart_tran_bank0[6] --------------------+    |     |     |     |     |     |
//cart_tran_bank0[5] -------------------------+     |     |     |     |     |
//cart_tran_bank0[4] -------------------------------+     |     |     |     |
//cart_tran_bank1[6] -------------------------------------+     |     |     | cart_tran_bank1_dir=1'b1 //output
//cart_tran_bank1[7] -------------------------------------------+     |     |
//cart_tran_pin30    -------------------------------------------------+     | cart_tran_pin30_dir=1'b1, cart_pin30_pwroff_reset=1'b1 (GPIO USE)
//cart_tran_pin31    -------------------------------------------------------+ cart_tran_pin31_dir=1'b0 / 1'b1 
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------++------------------
//                                                      C O N F I G U R A T I O N   A                                                                                || CONFIGURATION  B
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------++------------------
//     DEV TYPE          0                   1                    2                   3                   4                     5                 6                  || 16         
//     PIN_NAME         SNAC DISABLED        DB15		          NES                 SNES                PCENGINE(2BTN)        PCENGINE(6BTN)    PCENGINE(MULTITAP) || PSX
//USB3       SNAC                                                                                                                                                    || [NOT IMPLEMENTED]
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------||------------------
//VBUS 		 	                             +5V                  +5V                 +5V                 +5V                   +5V               +5V                || +5V                 
//D-		 OUT1      	                     CLK(O)               CLK_1(O)            CLK_1(O)            CLR(O)(*)             CLR(O)(*)         CLR(O)(*)          || AT1(O)     
//D+		 OUT2      	                     LAT(O)               LAT(O)              LAT(O)              SEL(O)(*)             SEL(O)(*)         SEL(O)(*)          || AT2(O)     
//GND 		 	                             GND                  GND                 GND                 GND                   GND               GND                || GND     
//RX-		  IO3                            DAT(I)               D0_1(I)             D0_1(I)             D2 (I)(*)             D2 (I)(*)         D2 (I)(*)          || CLK(O)          +
//RX+		  IN4                                                 D4_2(I)             IO_2(I)             D0 (I)(*)             D0 (I)(*)         D0 (I)(*)          || DAT(I)      
//GND_DRAIN	  IO5                                                 CLK_2(O)            CLK_2(O)                                                                       || ACK(I)          +
//TX-		  IO6                            DAT(I)(1)            D3_2(I)             D3_2(I)             D1 (I)(*)             D1 (I)(*)         D1 (I)(*)          || CMD(O)          +    
//TX+		  IN7                            D0_2(I)              D0_2(I)             D3 (I)(*)           D3 (I)(*)             D3 (I)(*)         D3 (I)(*)          || IRQ10(I)      
//
//(1) Alternate output of DAT (for male-to-male extension cables which cross Tx,Rx lines)

//(*) Needs a specific cable harness for use MiSTer SNAC adapter with the Pocket:
//  SNAC      PCENGINE     POCKET
// ADAPTER    FUNCTION     SNAC
//   D-    -> D0       ->  RX+    IN4
//   D+    -> D1       ->  TX-    IO6 (IN)
//   RX-   -> D2       ->  RX-    IO3 (IN)
//   RX+   -> CLR      ->  D-     OUT1
//   GND_D -> D3       ->  TX+    IN7 
//   TX-   -> SEL      ->  D+     OUT2
`default_nettype none
`timescale 1ns / 1ps

module openFPGA_Pocket_Analogizer_SNAC #(parameter MASTER_CLK_FREQ=50_000_000)
(
    input wire i_clk, //Core Master Freq.
    input wire i_rst, //Core general reset
    input wire conf_AB, //0 conf. A(default), 1 conf. B (see graph above)
    input wire [4:0] game_cont_type, //0-15 Conf. A, 16-31 Conf. B
    //input wire [2:0] game_cont_sample_rate, //0 compatibility mode (slowest), 1 normal mode, 2 fast mode, 3 superfast mode
    //PSX rumble interface joy1, joy2
    input [1:0] i_VIB_SW1,  //  Vibration SW  VIB_SW[0] Small Moter OFF 0:ON  1:
                                //VIB_SW[1] Bic Moter   OFF 0:ON  1(Dualshook Only)
	input [7:0] i_VIB_DAT1,  //  Vibration(Bic Moter)Data   8'H00-8'HFF (Dualshook Only)
    input [1:0] i_VIB_SW2,
	input [7:0] i_VIB_DAT2,  
    output reg [15:0] p1_btn_state,
    output reg [31:0] p1_joy_state,
    output reg [15:0] p2_btn_state,
    output reg [31:0] p2_joy_state,
    output reg [15:0] p3_btn_state,
    output reg [15:0] p4_btn_state,
    output reg busy,    
    //SNAC Pocket cartridge port interface (see graph above)   
    output reg [7:4] CART_BK0_OUT,
    input  wire [7:4] CART_BK0_IN,
    output reg CART_BK0_DIR, 
    output reg [7:6] CART_BK1_OUT_P76,
    output reg CART_PIN30_OUT,
    input  wire CART_PIN30_IN,
    output reg CART_PIN30_DIR, 
    output reg CART_PIN31_OUT,
    input  wire CART_PIN31_IN,
    output reg CART_PIN31_DIR,
    //JVS profile (jvs_vendor.bin data slot, see analogizer/jvs/scripts/jvs_vendor.py), written in clk_74a:
    //one 32-bit word per write, file byte 0 in bits [31:24]
    input  wire        clk_74a,
    input  wire        vendor_wr,
    input  wire [5:0]  vendor_wr_addr,
    input  wire [31:0] vendor_wr_data,
    input  wire [1:0]  recoil_mode,       //gun recoil: 0 off, 1 single, 2 auto (machine gun)
    //debug
    output wire [3:0] DBG_TX,
    output wire o_stb
); 
    //
    logic SNAC_OUT1 /* synthesis preserve */; //cart_tran_bank1[6]                                           D-
    logic SNAC_OUT2 /* synthesis preserve */; //cart_tran_bank1[7]                                           D+
    logic SNAC_IO3_A /* synthesis preserve */;//Conf.A: cart_tran_bank0[4] (in),  Conf.B: pin30(out)         RX- 
    logic SNAC_IO3_B /* synthesis preserve */;//Conf.B: cart_tran_bank0[4] (in),  Conf.B: pin30(out)         RX-
    logic SNAC_IN4 /* synthesis preserve */;  //cart_tran_bank0[7]                                           RX+
    logic SNAC_IO5_A /* synthesis preserve */;//Conf.A: pin30(out),               Conf.B: cart_tran_bank1[6] GND_D
	 logic SNAC_IO5_B /* synthesis preserve */;//Conf.A: pin30(out),               Conf.B: cart_tran_bank1[6] GND_D
    logic SNAC_IO6_A /* synthesis preserve */;//Conf.A: pin31(in),                Conf.B: pin31(out)         TX-
    logic SNAC_IO6_B /* synthesis preserve */;//Conf.A: pin31(in),                Conf.B: pin31(out)         TX-
    logic SNAC_IN7 /* synthesis preserve */;   //cart_tran_bank0[5]                                          TX+
    
    //calculate step sizes for fract clock enables
    // localparam pce_compat_polling_freq    =  20_000; //  20_000 / 5 =   4K samples/sec PCE
    localparam pce_normal_polling_freq    =  40_000; //  40_000 / 5 =   8K samples/sec PCE
    localparam pce_fast_polling_freq      =  80_000; //  80_000 / 5 =  16K samples/sec PCE
    // localparam pce_very_fast_polling_freq = 100_000; // 100_000 / 5 =  20K samples/sec PCE

    // localparam Compat_60Hz_polling_freq        =   1_080; //  
    // localparam Compat_120Hz_polling_freq        =  2_160; //  
    localparam snes_compat_polling_freq        =   50_000; //                                           
    // localparam serlatch_compat_polling_freq    =   100_000; //  100_000 / 25 =  4K samples/sec DB15    100_000 / 18 =  5.55K samples/sec NES/SNES
    localparam serlatch_normal_polling_freq    =   200_000; //  200_000 / 25 =  8K samples/sec DB15    200_000 / 18 = 11.11K samples/sec NES/SNES
    localparam serlatch_fast_polling_freq      =   400_000; //  400_000 / 25 = 16K samples/sec DB15    400_000 / 18 = 22.22K samples/sec NES/SNES
    // localparam serlatch_very_fast_polling_freq = 1_000_000; //1_000_000 / 25 = 32K samples/sec DB15  1_000_000 / 18 = 55.55K samples/sec NES/SNES

    localparam psx_normal_polling_freq = 125_000;
    localparam psx_fast_polling_freq = 250_000;
    localparam psx_ultra_fast_polling_freq = 500_000;
    localparam psx_multitap_polling_freq = 1_000_000;

    // localparam uart_dbg_freq = 500_000 * 16; //115_200 * 16;
    localparam uart_dbg_freq = 1_000_000; //115_200 * 16;
    localparam [64:0] uart_dbg_pstep_   = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) * uart_dbg_freq * 2) / 1000;
    localparam [32:0] uart_dbg_pstep    = uart_dbg_pstep_[32:0];


    //the FSM is clocked 2x the polling freq.
    localparam [32:0] MAX_INT = 33'h0ffffffff;
    // localparam [64:0] pce_compat_pstep_ = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) * pce_compat_polling_freq * 2) / 1000;
    // localparam [32:0] pce_compat_pstep  = pce_compat_pstep_[32:0];
    localparam [64:0] pce_normal_pstep_ = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) * pce_normal_polling_freq * 2) / 1000;
    localparam [32:0] pce_normal_pstep  = pce_normal_pstep_[32:0];
    localparam [64:0] pce_fast_pstep_   = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) * pce_fast_polling_freq * 2) / 1000;
    localparam [32:0] pce_fast_pstep    = pce_fast_pstep_[32:0];

    localparam [64:0] psx_fast_pstep_   = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) * psx_fast_polling_freq * 2) / 1000;
    localparam [32:0] psx_fast_pstep    = psx_fast_pstep_[32:0];
    localparam [64:0] psx_normal_pstep_   = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) * psx_normal_polling_freq * 2) / 1000;
    localparam [32:0] psx_normal_pstep    = psx_normal_pstep_[32:0];
    localparam [64:0] psx_ultra_fast_pstep_   = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) * psx_ultra_fast_polling_freq * 2) / 1000;
    localparam [32:0] psx_ultra_fast_pstep    = psx_ultra_fast_pstep_[32:0];
    localparam [64:0] psx_multitap_pstep_   = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) * psx_multitap_polling_freq * 2) / 1000;
    localparam [32:0] psx_multitap_pstep    = psx_multitap_pstep_[32:0];
    // localparam [64:0] pce_very_fast_pstep_   = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) *pce_very_fast_polling_freq * 2) / 1000;
    // localparam [32:0] pce_very_fast_pstep    = pce_very_fast_pstep_[32:0];

    // localparam [64:0] serlatch_compat_pstep_   = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) * serlatch_compat_polling_freq * 2) / 1000;
    // localparam [32:0] serlatch_compat_pstep    = serlatch_compat_pstep_[32:0];
    localparam [64:0] serlatch_normal_pstep_   = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) * serlatch_normal_polling_freq * 2) / 1000;
    localparam [32:0] serlatch_normal_pstep    = serlatch_normal_pstep_[32:0];
    localparam [64:0] serlatch_fast_pstep_   = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) * serlatch_fast_polling_freq * 2) / 1000;
    localparam [32:0] serlatch_fast_pstep    = serlatch_fast_pstep_[32:0];
    // localparam [64:0] serlatch_very_fast_pstep_   = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) * serlatch_very_fast_polling_freq * 2) / 1000;
    // localparam [32:0] serlatch_very_fast_pstep    = serlatch_very_fast_pstep_[32:0];
    
    localparam [64:0] snes_compat_pstep_   = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) * snes_compat_polling_freq  * 2) / 1000;
    localparam [32:0] snes_compat_pstep    = snes_compat_pstep_[32:0];

    // localparam [64:0] Compat_60Hz_pstep_   = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) * Compat_60Hz_polling_freq  * 2) / 1000;
    // localparam [32:0] Compat_60Hz_pstep    = Compat_60Hz_pstep_[32:0];
    // localparam [64:0] Compat_120Hz_pstep_   = ((MAX_INT / (MASTER_CLK_FREQ / 1000)) * Compat_120Hz_polling_freq  * 2) / 1000;
    // localparam [32:0] Compat_120Hz_pstep    = Compat_120Hz_pstep_[32:0];
    
    //Supported game controller types
    localparam GC_DISABLED        = 5'h0;
    localparam GC_DB15            = 5'h1;
    localparam GC_NES             = 5'h2;
    localparam GC_SNES            = 5'h3;
    localparam GC_PCE_2BTN        = 5'h4;
    localparam GC_PCE_6BTN        = 5'h5;
    localparam GC_PCE_MULTITAP    = 5'h6;
    localparam GC_DB15_FAST       = 5'h9;
    localparam GC_SNES_SWAP       = 5'hB;
    localparam GC_PSX             = 5'h10; //16 PSX 125KHz
    localparam GC_PSX_FAST        = 5'h11; //17 PSX 250KHz
    localparam GC_PSX_ANALOG      = 5'h12; //16 PSX 125KHz
    localparam GC_PSX_ANALOG_FAST = 5'h13; //17 PSX 250KHz
    localparam GC_JVS             = 5'h15; //21 JVS (RS485 adapter on the SNAC port, Conf. A pins; 0x14 is the SNES core's PSX lightgun)

    //Configuration:
    localparam CONF_A = 1'b0;
    localparam CONF_B = 1'b1;

    reg conf_AB_r;
    reg [4:0] game_cont_type_r;
    // reg [2:0] game_cont_sample_rate_r;
    reg [32:0] strobe_step_size;
    reg reset_on_change;

    always @(posedge i_clk) begin
        //register SNAC settings
        conf_AB_r <= conf_AB;
        game_cont_type_r <= game_cont_type;
        //game_cont_sample_rate_r <= game_cont_sample_rate;

        //detect change of SNAC settings and reset clock divider and set new settings
        reset_on_change <= 1'b0;
        //if(i_rst || (game_cont_type_r != game_cont_type) || (game_cont_sample_rate_r != game_cont_sample_rate)) begin
        if(i_rst || (game_cont_type_r != game_cont_type)) begin
            reset_on_change <= 1'b1;
        end
    end

    reg serlat_ena;
    reg pce_ena;
    reg psx_ena;
    reg jvs_ena;

    always @(posedge i_clk) jvs_ena <= (game_cont_type == GC_JVS);

    always @(posedge i_clk) begin
        case (game_cont_type)
            GC_PSX, GC_PSX_ANALOG: begin
                psx_ena    <= 1'b1;
                strobe_step_size <= psx_normal_pstep;   
            end
            GC_PSX_FAST, GC_PSX_ANALOG_FAST: begin
                psx_ena    <= 1'b1;
                strobe_step_size <= psx_fast_pstep;   
            end
            GC_DB15: begin 
                serlat_ena <= 1'b1;
                strobe_step_size <= serlatch_normal_pstep;    
                // case (game_cont_sample_rate)
                //     0: begin strobe_step_size <= serlatch_compat_pstep;    end
                //     1: begin strobe_step_size <= serlatch_normal_pstep;    end
                //     2: begin strobe_step_size <= serlatch_fast_pstep;      end
                //     // 3: begin strobe_step_size <= serlatch_very_fast_pstep; end 
                //     // 4: begin strobe_step_size <= snes_compat_pstep;        end 
                //     // 5: begin strobe_step_size <= Compat_60Hz_pstep;        end
                //     // 6: begin strobe_step_size <= Compat_120Hz_pstep;       end
                //     default: begin strobe_step_size <= serlatch_compat_pstep ;  end
                // endcase
            end
            GC_DB15_FAST: begin 
                serlat_ena <= 1'b1;
                strobe_step_size <= serlatch_fast_pstep;    
            end
            GC_NES, GC_SNES, GC_SNES_SWAP: begin 
                serlat_ena <= 1'b1;
                strobe_step_size <= snes_compat_pstep;
                // case (game_cont_sample_rate)
                //     0: begin strobe_step_size <= serlatch_compat_pstep ;   end
                //     1: begin strobe_step_size <= serlatch_normal_pstep;    end
                //     2: begin strobe_step_size <= serlatch_fast_pstep;      end
                //     // 3: begin strobe_step_size <= serlatch_very_fast_pstep; end 
                //     // 4: begin strobe_step_size <= snes_compat_pstep;        end 
                //     // 5: begin strobe_step_size <= Compat_60Hz_pstep;        end
                //     // 6: begin strobe_step_size <= Compat_120Hz_pstep;       end
                //     default: begin strobe_step_size <= serlatch_compat_pstep ;  end
                // endcase
            end
            GC_PCE_2BTN, GC_PCE_6BTN: begin
                pce_ena    <= 1'b1;
                strobe_step_size <= pce_normal_pstep;
                // case (game_cont_sample_rate)
                //     0: begin strobe_step_size <= pce_compat_pstep;    end
                //     1: begin strobe_step_size <= pce_normal_pstep;    end
                //     2: begin strobe_step_size <= pce_fast_pstep;      end
                //     3: begin strobe_step_size <= pce_very_fast_pstep; end 
                //     default: begin strobe_step_size <= pce_compat_pstep;    end
                // endcase 
            end
            GC_PCE_MULTITAP: begin
                pce_ena    <= 1'b1;
                strobe_step_size <= pce_fast_pstep;
            end
            
            default: begin//disabled
                serlat_ena <= 1'b0;
                pce_ena    <= 1'b0;
                psx_ena    <= 1'b0;
                strobe_step_size <= 33'h0;
            end
        endcase
    end

    always @(posedge i_clk) begin
        case (conf_AB)         
            CONF_A: begin
                CART_BK0_DIR                   <= 1'b0;                                           //INPUT
                {SNAC_IN4,SNAC_IN7,SNAC_IO3_A} <= {CART_BK0_IN[7],CART_BK0_IN[5],CART_BK0_IN[4]}; //OUTPUT
                CART_BK1_OUT_P76                <= {SNAC_OUT2,SNAC_OUT1};                          //OUTPUT 
                CART_PIN30_DIR                 <= 1'b1;                                           //OUTPUT 
                CART_PIN30_OUT                 <= SNAC_IO5_A; 
                CART_PIN31_DIR                 <= 1'b0;                                           //INPUT
                SNAC_IO6_A                     <= CART_PIN31_IN;  
            end 
            CONF_B: begin 
                CART_BK0_DIR                    <= 1'b0;                                           //INPUT
                {SNAC_IN4,SNAC_IO5_B,SNAC_IN7} <= {CART_BK0_IN[7],CART_BK0_IN[6],CART_BK0_IN[5]}; //OUTPUT
                CART_BK1_OUT_P76                <= {SNAC_OUT2,SNAC_OUT1};                          //OUTPUT 
                CART_PIN30_DIR                 <= 1'b1;                                           //OUTPUT           
                CART_PIN30_OUT                 <= SNAC_IO3_B; 
                CART_PIN31_DIR                 <= 1'b1;                                           //OUTPUT
                CART_PIN31_OUT                 <= SNAC_IO6_B;
            end
        endcase
    end
                                               
    wire stb_clk /* synthesis keep */;
    clock_divider_fract ckdiv(
    .i_clk (i_clk),
    .i_rst(reset_on_change), //reset on polling freq change
    .i_step(strobe_step_size[31:0]),
    .o_stb (stb_clk)
    );

	 wire dbg_clk_w;
	 reg dbg_clk /* synthesis noprune */;
    clock_divider_fract dbgckdiv(
    .i_clk (i_clk),
    .i_rst(reset_on_change), //reset on polling freq change
    .i_step(uart_dbg_pstep[31:0]),
    .o_stb (dbg_clk_w)
    );
	 
	 always @(posedge i_clk) dbg_clk <= dbg_clk_w;

    assign o_stb = stb_clk;

    //PSX game controller for 1/2 players
    wire [15:0] psx_key1, psx_key2;
    wire [31:0] psx_joy1, psx_joy2;
    wire PSX_SNAC_OUT1 /* synthesis keep */;
    wire PSX_SNAC_OUT2 /* synthesis keep */;
    analogizer_psx #(.MASTER_CLK_FREQ(MASTER_CLK_FREQ)) psx (
        .i_clk(i_clk),
        .i_rst(reset_on_change),
        .i_ena(psx_ena),
        .i_stb(stb_clk),
        .key1(psx_key1),
        .joy1(psx_joy1),
        .key2(psx_key2),
        .joy2(psx_joy2),
        //PSX RUMBLE INTERFACE
        .i_VIB_SW1(i_VIB_SW1), .i_VIB_DAT1(i_VIB_DAT1), .i_VIB_SW2(i_VIB_SW2), .i_VIB_DAT2(i_VIB_DAT2), 
        //PSX EXTERNAL INTERFACE
        .PSX_CLK(SNAC_IO3_B),
        .PSX_DAT(SNAC_IN4),
        .PSX_CMD(SNAC_IO6_B),
        .PSX_ATT1(PSX_SNAC_OUT1),
        .PSX_ATT2(PSX_SNAC_OUT2),
        .PSX_ACK(SNAC_IO5_B),
        .DBG_TX(DBG_TX)
    );
    //assign PSX_SNAC_OUT2 = 1'b1;

    //DB15/NES/SNES game controller
    wire [15:0] sl_p1 /* synthesis keep */;
    wire [15:0] sl_p2 /* synthesis keep */;
    wire SERLAT_SNAC_OUT1 /* synthesis keep */;
    wire SERLAT_SNAC_OUT2 /* synthesis keep */;
    //wire SERLAT_SNAC_IO5_A /* synthesis keep */;
    serlatch_game_controller #(.MASTER_CLK_FREQ(MASTER_CLK_FREQ)) slgc
    (
        .i_clk(i_clk),
        .i_rst(reset_on_change),
        .game_controller_type(game_cont_type[3:0]), //0x1 DB15, 0x2 NES, 0x3 SNES, 0x9 DB15 FAST, 0XB SNES SWAP A,B<->X,Y
        .i_stb(stb_clk),
        .p1_btn_state(sl_p1),
        .p2_btn_state(sl_p2),
        .busy(),
        //SNAC Game controller interface
        .o_clk(SERLAT_SNAC_OUT1), //shared for 2 controllers
        .o_clk2(SNAC_IO5_A),
        .o_lat(SERLAT_SNAC_OUT2), //shared for 2 controllers
        .i_dat1((game_cont_type == 5'h1 || game_cont_type == 5'h9) ? SNAC_IO3_A & SNAC_IO6_A : SNAC_IO3_A ), //data from controller 1
        .i_dat2(SNAC_IN7)  //data from controller 2
    );

    //PCENGINE game controller
    wire [15:0] pce_p1 /* synthesis keep */;
    wire PCE_SNAC_OUT1 /* synthesis keep */;
    wire PCE_SNAC_OUT2 /* synthesis keep */;

    pcengine_game_controller #(.MASTER_CLK_FREQ(MASTER_CLK_FREQ), .PULSE_CLR_LINE(1'b1)) pcegc1
    (
        .i_clk(i_clk),
        .i_rst(reset_on_change),
        .game_controller_type(game_cont_type[3:0]), //0X4 2btn, 0X5 6btn
        .i_stb(stb_clk),
        .player_btn_state(pce_p1),
        .busy(),
        //SNAC Game controller interface
        .o_clr(PCE_SNAC_OUT1), //shared for 2 controllers
        .o_sel(PCE_SNAC_OUT2), //shared for 2 controllers
        .i_dat({SNAC_IN7,SNAC_IO3_A,SNAC_IO6_A,SNAC_IN4}) //data from controller
    );

wire [15:0] pce_multitap_p1, pce_multitap_p2, pce_multitap_p3, pce_multitap_p4;
wire PCE_MULTITAP_SNAC_OUT1, PCE_MULTITAP_SNAC_OUT2;

pcengine_game_controller_multitap #(.MASTER_CLK_FREQ(MASTER_CLK_FREQ)) pcegmutitap
(
        .i_clk(i_clk),
        .i_rst(reset_on_change),
        .game_controller_type(game_cont_type[3:0]), //0x6 multitap
        .i_stb(stb_clk),
        .player1_btn_state(pce_multitap_p1),
        .player2_btn_state(pce_multitap_p2),
        .player3_btn_state(pce_multitap_p3),
        .player4_btn_state(pce_multitap_p4),
        .player5_btn_state(),
        .busy(),
    //SNAC Game controller interface
    .o_clr(PCE_MULTITAP_SNAC_OUT1), //shared for 2 controllers
    .o_sel(PCE_MULTITAP_SNAC_OUT2), //shared for 2 controllers
    .i_dat({SNAC_IN7,SNAC_IO3_A,SNAC_IO6_A,SNAC_IN4}) //data from controller
);

    //JVS I/O board (analogizer/jvs submodule, sduponch/jvs): init, addressing and polling in hardware.
    //TX on SNAC OUT1, RS485 driver enable on OUT2, RX on IN4, SENSE on IN7 (Conf. A).
    wire [15:0] jvs_sw1, jvs_sw2;
    wire        jvs_coin1, jvs_coin2;
    wire        JVS_UART_TX, JVS_485_DIR;
    wire [jvs_node_info_pkg::MAX_JVS_NODES*128-1:0] jvs_node_analog;
    logic [23:0] p_output_digital [2];        //OUTPUT1 bits of P1, P2 {byte 3, 2, 1} (profile base, recoil)

    //---- JVS profile: 256 bytes, dual clock BRAM (bridge writes in clk_74a, jvs_ctrl reads in i_clk, byte valid
    //2 cycles after vendor_rd_addr). All zero without jvs_vendor.bin: standard init, no MAINID / vendor frame.
    (* ramstyle = "M10K" *) logic [31:0] vendor_mem [0:63];
    initial for (int i = 0; i < 64; i++) vendor_mem[i] = 32'h0;
    always @(posedge clk_74a) if (vendor_wr) vendor_mem[vendor_wr_addr] <= vendor_wr_data;
    wire  [7:0]  vendor_rd_addr;
    logic [31:0] vendor_rd_word;
    logic [1:0]  vendor_rd_lane;
    always @(posedge i_clk) begin
        vendor_rd_word <= vendor_mem[vendor_rd_addr[7:2]];
        vendor_rd_lane <= vendor_rd_addr[1:0];
    end
    wire  [7:0]  vendor_rd_data = vendor_rd_word[8*(3 - vendor_rd_lane) +: 8];

    //the fixed block at the end of the table, kept in registers too:
    //  0xD8..0xDF player axes P1 X, P1 Y, P2 X, P2 Y ...: bit 7 defined, bits 6..4 node - 1, bit 3 inverted, bits 2..0 channel
    //  0xE8..0xEB gun trigger (recoil trigger) mask of P1, P2: 2 bytes each in JVS order (switch bytes 1, 2)
    //  0xF0 bit 0 output profile valid, 0xF1..0xF3 OUTPUT1 base, 0xF4..0xF6 / 0xF7..0xF9 recoil mask of P1 / P2
    logic [31:0] prof_words_74 [10] = '{default: 32'h0};     //words 0x36..0x3F: bytes 0xD8..0xFF
    always @(posedge clk_74a) if (vendor_wr && vendor_wr_addr >= 6'h36) prof_words_74[vendor_wr_addr - 6'h36] <= vendor_wr_data;
    logic [7:0] prof [40];                                    //bytes 0xD8..0xFF (quasi static: re-registered in i_clk)
    always @(posedge i_clk) for (int b = 0; b < 40; b++) prof[b] <= prof_words_74[b / 4][8 * (3 - b % 4) +: 8];
    wire        dout_valid  = prof[8'hF0 - 8'hD8][0];
    wire [23:0] dout_base   = {prof[8'hF3 - 8'hD8], prof[8'hF2 - 8'hD8], prof[8'hF1 - 8'hD8]};
    wire [23:0] dout_recoil [2];
    assign dout_recoil[0] = {prof[8'hF6 - 8'hD8], prof[8'hF5 - 8'hD8], prof[8'hF4 - 8'hD8]};
    assign dout_recoil[1] = {prof[8'hF9 - 8'hD8], prof[8'hF8 - 8'hD8], prof[8'hF7 - 8'hD8]};
    wire [15:0] trig_mask [2];
    assign trig_mask[0] = {prof[8'hE8 - 8'hD8], prof[8'hE9 - 8'hD8]};
    assign trig_mask[1] = {prof[8'hEA - 8'hD8], prof[8'hEB - 8'hD8]};
    logic [3:0]  axis_defined;                                //[2p] player p X, [2p+1] player p Y (P1, P2)
    logic [15:0] axis_value [4];
    always_comb begin
        for (int a = 0; a < 4; a++) begin                       //channel node * 8 + CH of node_analog
            axis_defined[a] = prof[a][7] && (prof[a][6:4] < jvs_node_info_pkg::MAX_JVS_NODES);
            axis_value[a]   = !axis_defined[a] ? 16'h8000 :
                              prof[a][3] ? ~jvs_node_analog[{prof[a][6:4], prof[a][2:0]}*16 +: 16]
                                         :  jvs_node_analog[{prof[a][6:4], prof[a][2:0]}*16 +: 16];
        end
    end

    //a profile loaded (or reloaded from the Pocket menu) while running: the controller is held in reset during the
    //transfer and 10 ms after, so that the JVS init restarts with it
    localparam int VENDOR_LOAD_HOLD = 742_500;               //10 ms at 74.25 MHz
    logic [$clog2(VENDOR_LOAD_HOLD+1)-1:0] vendor_load_cnt = '0;
    logic vendor_loading_74 = 1'b0;
    always @(posedge clk_74a) begin
        if (vendor_wr)                 vendor_load_cnt <= VENDOR_LOAD_HOLD[$bits(vendor_load_cnt)-1:0];
        else if (vendor_load_cnt != 0) vendor_load_cnt <= vendor_load_cnt - 1'b1;
        vendor_loading_74 <= vendor_wr || (vendor_load_cnt != 0);
    end
    logic [1:0] vendor_loading_s = 2'b00;
    always @(posedge i_clk) vendor_loading_s <= {vendor_loading_s[0], vendor_loading_74};

    jvs_ctrl #(
        .MASTER_CLK_FREQ(MASTER_CLK_FREQ),
        .HAS_LINK_DETECT(0)
    ) jvs_inst (
        .i_clk(i_clk),
        .i_rst(reset_on_change || vendor_loading_s[1]),
        .i_ena(jvs_ena),
        .i_stb(1'b0),
        .i_uart_rx(SNAC_IN4),
        .o_uart_tx(JVS_UART_TX),
        .i_sense(SNAC_IN7),
        .i_link(~SNAC_IO3_A),
        .o_rx485_dir(JVS_485_DIR),
        .o_bus_state(),
        .player1_input_switch(jvs_sw1),
        .player2_input_switch(jvs_sw2),
        .player3_input_switch(),
        .player4_input_switch(),
        .analog_ch1(), .analog_ch2(), .analog_ch3(), .analog_ch4(),
        .analog_ch5(), .analog_ch6(), .analog_ch7(), .analog_ch8(),
        .node_analog(jvs_node_analog),
        .has_screen_pos(),
        .screen_pos_x(),
        .screen_pos_y(),
        .coin_count(),
        .coin1(jvs_coin1),
        .coin2(jvs_coin2),
        .coin3(),
        .coin4(),
        .coin_status(),
        .p1_output_digital(p_output_digital[0]),
        .p2_output_digital(p_output_digital[1]),
        .p3_output_digital(24'd0),
        .p4_output_digital(24'd0),
        .output_analog(128'd0),
        .output_node(8'd0),
        .jvs_data_ready(),
        .jvs_nodes(),
        .node_name_rd_data(),
        .node_name_rd_addr('0),
        .vendor_rd_addr(vendor_rd_addr),
        .vendor_rd_data(vendor_rd_data)
    );

    //---- guns: a player's trigger is any switch of its profile trigger mask; its position the profile axes.
    //Recoil, made here from the trigger (the games do not drive one), the player's recoil mask added to the profile's
    //OUTPUT1 base while it is on:
    //  single  one 50 ms pulse per trigger press
    //  auto    while the trigger is held, 50 ms on / 150 ms off (5 shots a second), from the press on
    //  off     never (the profile's base value only)
    function automatic [15:0] jvs_order(input [15:0] sw);    //switches back in JVS order (switch bytes 1, 2)
        jvs_order = {sw[15], sw[14], sw[0], sw[1], sw[2], sw[3], sw[4], sw[5],
                     sw[6], sw[7], sw[8], sw[9], sw[10], sw[11], sw[12], sw[13]};
    endfunction
    wire [1:0] gun_trigger = {|(jvs_order(jvs_sw2) & trig_mask[1]), |(jvs_order(jvs_sw1) & trig_mask[0])};
    wire [1:0] gun_defined = {|trig_mask[1], |trig_mask[0]};
    function automatic [15:0] pocket_order(input [15:0] j);  //inverse of jvs_order: a JVS-order mask in the Pocket's layout
        pocket_order = {j[15], j[14], j[0], j[1], j[2], j[3], j[4], j[5],
                        j[6], j[7], j[8], j[9], j[10], j[11], j[12], j[13]};
    endfunction
    //the trigger's switches are the lightgun's fire only: also mapping them to a pad button sent the Super Scope's
    //pause (Y, push 1 in jvs_gun_p1.bin) with each shot, which opens the calibration screen
    wire [15:0] jvs_sw1_pad = jvs_sw1 & ~pocket_order(trig_mask[0]);
    wire [15:0] jvs_sw2_pad = jvs_sw2 & ~pocket_order(trig_mask[1]);
    localparam integer RECOIL_ON  = MASTER_CLK_FREQ / 20;    //50 ms
    localparam integer RECOIL_CYC = MASTER_CLK_FREQ / 5;     //200 ms: the auto mode's period
    localparam integer RW = $clog2(RECOIL_CYC + 1);
    reg [RW-1:0] recoil_cnt [2];
    reg [1:0] gun_trigger_d;
    wire      recoil_auto = (recoil_mode == 2'd2);
    always @(posedge i_clk) begin
        gun_trigger_d <= gun_trigger;
        for (int p = 0; p < 2; p++) begin
            if (recoil_mode == 2'd0) recoil_cnt[p] <= '0;
            else if ((gun_trigger[p] && !gun_trigger_d[p]) || (recoil_auto && gun_trigger[p] && recoil_cnt[p] == 0))
                recoil_cnt[p] <= recoil_auto ? RW'(RECOIL_CYC) : RW'(RECOIL_ON);
            else if (recoil_cnt[p] != 0) recoil_cnt[p] <= recoil_cnt[p] - 1'b1;
            p_output_digital[p] <= !dout_valid ? 24'h000000 :
                                   (dout_base | (recoil_cnt[p] > (recoil_auto ? RW'(RECOIL_CYC - RECOIL_ON) : RW'(0)) ? dout_recoil[p] : 24'h000000));
        end
    end
    //P1's gun (axes defined in the profile) as the Pocket's left stick, absolute, for the SNES core's lightgun
    //(rtl/lightgun.sv: X 0..255 across the screen, Y 0..255 scaled to the frame's height), which fires with A (F)
    wire [7:0] gun_x8 = axis_value[0][15:8];
    wire [7:0] gun_y8 = axis_value[1][15:8];
    wire       p1_gun = axis_defined[0] || axis_defined[1];

    //jvs_ctrl gives the Pocket's key layout (bits 3:0 the D-pad, 4..9 push 1..6, 14 SERVICE, 15 START). The SNES core reads
    //A bit 4, B 5, X 6, Y 7, L 8, R 9, SELECT 14, START 15. Panel of the Nintendo Super System: top row push 1..3 -> Y X L,
    //bottom row push 4..6 -> B A R; SERVICE -> SELECT; the coins are not used (no coin on a console).
    function automatic [15:0] jvs_to_snes(input [15:0] sw);
        jvs_to_snes = {sw[15], sw[14], 4'b0000, sw[9], sw[6], sw[4], sw[5], sw[7], sw[8], sw[3:0]};
    endfunction

    always @(*) begin
        p1_joy_state = 32'h80808080; //analog stick neutral position value
        p2_joy_state = 32'h80808080; //analog stick neutral position value

        case(game_cont_type)
        GC_DISABLED: begin
            SNAC_OUT1 = 1'b0;
            SNAC_OUT2 = 1'b0;
            p1_btn_state = 16'h0; 
            p2_btn_state = 16'h0;
            p3_btn_state = 16'h0; 
            p4_btn_state = 16'h0;
        end
        GC_DB15, GC_DB15_FAST, GC_NES, GC_SNES, GC_SNES_SWAP: begin
            SNAC_OUT1 = SERLAT_SNAC_OUT1;
            SNAC_OUT2 = SERLAT_SNAC_OUT2;            
            p1_btn_state = sl_p1;
            p2_btn_state = sl_p2; 
            p3_btn_state = 16'h0; 
            p4_btn_state = 16'h0;

        end
        GC_PCE_2BTN, GC_PCE_6BTN: begin
            SNAC_OUT1 = PCE_SNAC_OUT1;
            SNAC_OUT2 = PCE_SNAC_OUT2;
            p1_btn_state = pce_p1; 
            p2_btn_state = 16'h0;
            p3_btn_state = 16'h0; 
            p4_btn_state = 16'h0;
        end
        GC_PCE_MULTITAP: begin
            SNAC_OUT1 = PCE_MULTITAP_SNAC_OUT1;
            SNAC_OUT2 = PCE_MULTITAP_SNAC_OUT2;
            p1_btn_state = pce_multitap_p1; 
            p2_btn_state = pce_multitap_p2;
            p3_btn_state = pce_multitap_p3; 
            p4_btn_state = pce_multitap_p4;
        end
        GC_JVS: begin
            SNAC_OUT1 = JVS_UART_TX;
            SNAC_OUT2 = JVS_485_DIR;
            p1_btn_state = jvs_to_snes(jvs_sw1_pad);
            p2_btn_state = jvs_to_snes(jvs_sw2_pad);
            if (p1_gun) p1_joy_state = {16'h8080, gun_y8, gun_x8};
            if (gun_defined[0]) p1_btn_state[4] = gun_trigger[0];        //the gun trigger is the lightgun's fire (A)
            p3_btn_state = 16'h0;
            p4_btn_state = 16'h0;
        end
        GC_PSX, GC_PSX_ANALOG, GC_PSX_FAST, GC_PSX_ANALOG_FAST: begin
            SNAC_OUT1 = PSX_SNAC_OUT1;
            SNAC_OUT2 = PSX_SNAC_OUT2;
            p1_btn_state = psx_key1; 
            p1_joy_state = psx_joy1;
            p2_btn_state = psx_key2; 
            p2_joy_state = psx_joy2;
            p3_btn_state = 16'h0;
            p4_btn_state = 16'h0;
        end
        default: begin
            SNAC_OUT1 = 1'b0;
            SNAC_OUT2 = 1'b0;
            p1_btn_state = 16'h0; 
            p2_btn_state = 16'h0;
            p3_btn_state = 16'h0; 
            p4_btn_state = 16'h0;
        end
        endcase
    end
endmodule