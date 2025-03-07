`timescale 1ns / 1ps

module spi_slave #(
    parameter HEADER_WIDTH = 16,
    parameter PAYLOAD_WIDTH = 128,
    parameter TOTAL_WIDTH = HEADER_WIDTH + PAYLOAD_WIDTH,// Total data width (16 + 128 = 144 bits)
    parameter MODE = 0
)(
    // System signals
    input                           clk, // System clock
    input                           rst_n,
    
    // SPI interface signals
    input                           spi_clk, // SPI clock (SCLK) from master
    input                           spi_cs_n, // Chip select (CS/SS) - active low
    input                           spi_mosi,// Master Out Slave In (MOSI)
    output                          spi_miso, // Master In Slave Out (MISO)
    
    // Parallel data interface - TX (to master)
    input  [TOTAL_WIDTH-1:0]        tx_data, // Data to be transmitted
    input                           tx_send, // Indicates start of transfer
    output reg                      tx_ready, // Indicates ready to accept data
    
    // Parallel data interface - RX (from master)
    output reg [TOTAL_WIDTH-1:0]    rx_data,
    output reg                      rx_header_valid,  // Indicates header received
    output reg                      rx_payload_valid, // Indicates complete payload received
    output                          rx_complete // Indicates complete transfer
);

    // Mode 0: CPOL = 0, CPHA = 0
    // Mode 1: CPOL = 0, CPHA = 1
    // Mode 2: CPOL = 1, CPHA = 0
    // Mode 3: CPOL = 1, CPHA = 1

    // CPOL == 0 -> active high clock && CPOL == 1 -> active low clock
    localparam CPOL = (MODE == 1 || MODE == 3) ? 1'b1 : 1'b0;
    // CPHA == 0 -> data sampled on leading edge && CPHA == 1 -> data sampled on lagging edge
    localparam CPHA = (MODE == 2 || MODE == 3) ? 1'b1 : 1'b0;
    
    // MOSI state machine parameters
    localparam RX_IDLE = 2'b00, RX_START = 2'b01, RX_BITS = 2'b11, RX_COMPLETE = 2'b10;
    
    // MISO state machine parameters 
    localparam TX_IDLE = 2'b00, TX_START = 2'b01, TX_BITS = 2'b11, TX_COMPLETE = 2'b10;
    
    // Registers
    reg spi_clk_reg1;
    reg spi_clk_reg2;
    reg [8:0] tx_counter;
    reg [8:0] rx_counter;
    reg [TOTAL_WIDTH - 1: 0] rx_data_reg;
    reg [TOTAL_WIDTH - 1: 0] tx_data_reg;
    reg mosi_reg1;
    reg mosi_reg2;
    reg spi_cs_n_reg1, spi_cs_n_reg2;
    reg miso_data;
    reg tx_data_loaded;

    // state registers
    reg [1:0] rx_state, rx_next_state;
    reg [1:0] tx_state, tx_next_state;

    // wire
    wire spi_clk_leading_edge;
    wire spi_clk_trailing_edge;
    wire cs_n_falling_edge;
    wire cs_n_rising_edge;
    
    // Double reg for spi_clk to detect leading and lagging edges
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_clk_reg1 <= 1'b0;
            spi_clk_reg2 <= 1'b0;
        end else begin
            spi_clk_reg1 <= spi_clk;
            spi_clk_reg2 <= spi_clk_reg1;
        end
    end

    // Sync MOSI data with respect to reg-ed SPI clock
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mosi_reg1 <= 1'b0;
            mosi_reg2 <= 1'b0;
        end else begin
            mosi_reg1 <= spi_mosi;
            mosi_reg2 <= mosi_reg1;
        end
    end

    // Double reg the spi cs_n signal to detect falling edge
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_cs_n_reg1 <= 1'b1;
            spi_cs_n_reg2 <= 1'b1;
        end else begin
            spi_cs_n_reg1 <= spi_cs_n;
            spi_cs_n_reg2 <= spi_cs_n_reg1;
        end
    end
    
    // Leading edge detector 
    assign spi_clk_leading_edge = (CPOL == 0) ? (spi_clk_reg1 & !spi_clk_reg2) : (!spi_clk_reg1 & spi_clk_reg2);
    
    // Lagging edge detector
    assign spi_clk_trailing_edge = (CPOL == 0) ? (!spi_clk_reg1 & spi_clk_reg2) : (spi_clk_reg1 & !spi_clk_reg2);
    
    // CS Falling edge detector
    assign cs_n_falling_edge = spi_cs_n_reg2 & !spi_cs_n_reg1;

    // CS Rising edge detector
    assign cs_n_rising_edge = !spi_cs_n_reg2 & spi_cs_n_reg1;
    
    // RX_RX_COMPLETE
    assign rx_complete = cs_n_rising_edge;
    
    // MOSI state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state <= RX_IDLE;
        end 
        else begin
            rx_state <= rx_next_state;
        end
    end

    always @(*) begin
        rx_next_state = rx_state; // Default: maintain current state
        case (rx_state)
            RX_IDLE: begin
                rx_next_state = (cs_n_falling_edge) ? RX_START : RX_IDLE;
            end
            RX_START: begin
                if (CPHA == 0) begin
                    if (spi_clk_leading_edge) begin
                        rx_next_state = RX_BITS;
                    end
                end 
                else begin
                    if (spi_clk_trailing_edge) begin
                        rx_next_state = RX_BITS;
                    end
                end
            end
            RX_BITS: begin
                rx_next_state = (rx_counter == 0 && spi_clk_trailing_edge) ? RX_COMPLETE : RX_BITS;
            end
            RX_COMPLETE: begin
                rx_next_state = RX_IDLE;
            end
            default: begin
                rx_next_state = RX_IDLE;
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_data_reg <= 0;
            rx_header_valid <= 0;
            rx_payload_valid <= 0;
        end
        else begin
            case (rx_state)
                RX_IDLE: begin
                    rx_counter <= TOTAL_WIDTH;
                    rx_data_reg <= 0;
                    rx_header_valid <= 0;
                    rx_payload_valid <= 0;
                end
                RX_START: begin
                    if (CPHA == 0) begin
                        if (spi_clk_leading_edge) begin
                            rx_data_reg <= {rx_data_reg[TOTAL_WIDTH -1: 1], mosi_reg2};
                            rx_counter <= rx_counter - 1;
                        end
                    end 
                    else begin
                        if (spi_clk_trailing_edge) begin
                            rx_data_reg <= {rx_data_reg[TOTAL_WIDTH -1: 1], mosi_reg2};
                            rx_counter <= rx_counter - 1;
                        end
                    end
                end
                RX_BITS: begin
                    if (CPHA == 0) begin
                        if (spi_clk_leading_edge) begin
                            rx_data_reg <= {rx_data_reg[TOTAL_WIDTH - 2 : 0], mosi_reg2};
                            rx_counter <= rx_counter - 1;
                        end
                    end 
                    else begin
                        if (spi_clk_trailing_edge) begin
                            rx_data_reg <= {rx_data_reg[TOTAL_WIDTH - 2 : 0], mosi_reg2};
                            rx_counter <= rx_counter - 1;
                        end
                    end
                    if (rx_counter <= PAYLOAD_WIDTH - 1) begin
                        rx_header_valid <= 1;
                    end
                end
                RX_COMPLETE: begin
                    rx_data <= rx_data_reg;
                    rx_payload_valid <= 1;
                end
            endcase
        end
    end
    
    // MISO State machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state <= TX_IDLE;
        end 
        else begin
            tx_state <= tx_next_state;
        end
    end
    
    // Store the tx data into tx data reg only when module is ready
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_data_loaded <= 0;
            tx_data_reg <= 0;
        end
        else begin
            case (tx_state)
                TX_IDLE : begin
                    tx_data_loaded <= 0;

                    if (tx_send) begin
                        tx_data_reg <= tx_data;
                        tx_data_loaded <= 1;
                    end
                end 
                TX_COMPLETE : begin
                    tx_data_loaded <= 0;
                end
                default: begin
                    tx_data_reg <= tx_data_reg;
                end 
            endcase
        end
    end

    always @(*) begin
        tx_next_state = tx_state; // Default: maintain current state
        case (tx_state)
            TX_IDLE: begin
                tx_next_state = (cs_n_falling_edge) ? TX_START : TX_IDLE;
            end
            TX_START: begin
                tx_next_state = TX_BITS;
            end
            TX_BITS: begin
                tx_next_state = (tx_counter == 0 && spi_clk_trailing_edge) ? TX_COMPLETE : TX_BITS;
            end
            TX_COMPLETE: begin
                tx_next_state = TX_IDLE;
            end
            default: begin
                tx_next_state = TX_IDLE;
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_counter <= TOTAL_WIDTH - 1;
        end 
        else begin
            case (tx_state)
                TX_IDLE : begin
                    tx_ready <= 1;
                    if (CPHA == 0) begin
                        miso_data <= tx_data_reg[TOTAL_WIDTH - 1];
                        tx_counter <= TOTAL_WIDTH - 2;
                    end 
                    else begin
                        tx_counter <= TOTAL_WIDTH - 1;
                    end
                end
                TX_START : begin
                    tx_ready <= 0;
                    if (CPHA == 1) begin
                        if (spi_clk_leading_edge) begin
                            miso_data <= tx_data_reg[tx_counter];
                            tx_counter <= tx_counter - 1;
                        end
                    end 
                end
                TX_BITS : begin
                    tx_ready <= 0;
                    if (CPHA == 0) begin
                        if (spi_clk_trailing_edge) begin
                            miso_data <= tx_data_reg[tx_counter];
                            tx_counter <= tx_counter - 1;
                        end
                    end 
                    else begin
                        if (spi_clk_leading_edge) begin
                            miso_data <= tx_data_reg[tx_counter];
                            tx_counter <= tx_counter - 1;
                        end
                    end
                end
                TX_COMPLETE : begin
                    tx_ready <= 0;
                    tx_counter <= 0;
                end
            endcase
        end
    end
    
    // SPI miso
    assign spi_miso = (!spi_cs_n) ? miso_data : 1'bz;

endmodule
