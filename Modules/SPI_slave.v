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
    output reg                      spi_miso, // Master In Slave Out (MISO)
    
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

    // Registers
    reg spi_clk_reg1;
    reg spi_clk_reg2;
    reg [8:0] tx_counter;
    reg [8:0] rx_counter;
    reg [TOTAL_WIDTH - 1: 0] rx_data_reg;
    reg [TOTAL_WIDTH - 1: 0] tx_data_reg;
    reg rcv_tx_data;
    reg mosi_reg1;
    reg mosi_reg2;
    reg spi_cs_n_reg1, spi_cs_n_reg2;

    // wire
    wire spi_clk_leading_edge;
    wire spi_clk_trailing_edge;
    wire cs_n_falling_edge;
    
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

    // Storing TX data
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_data_reg <= {TOTAL_WIDTH{1'b0}};
            rcv_tx_data   <= 1'b0;
        end
        else begin
            rcv_tx_data <= tx_send;
            if (tx_send) begin
            tx_data_reg <= tx_data;
            end
        end 
    end 
    
    // Leading edge detector 
    assign spi_clk_leading_edge = (CPOL == 0) ? (spi_clk_reg1 & !spi_clk_reg2) : (!spi_clk_reg1 & spi_clk_reg2);
    
    // Lagging edge detector
    assign spi_clk_trailing_edge = (CPOL == 0) ? (!spi_clk_reg1 & spi_clk_reg2) : (spi_clk_reg1 & !spi_clk_reg2);
    
    // CS Falling edge detector
    assign cs_n_falling_edge = spi_cs_n_reg2 & ~spi_cs_n_reg1;

    // Reset logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_miso <= 1'b1;
            rx_header_valid <= 1'b0;
            rx_payload_valid <= 1'b0;
        end
        else if (spi_cs_n) begin
            spi_miso <= 1'b1;
            rx_header_valid <= 1'b0;
            rx_payload_valid <= 1'b0;
        end
    end

    // SPI MISO logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || spi_cs_n) begin
            spi_miso <= 1'b0;
            tx_counter <= TOTAL_WIDTH - 1; 
        end
        else begin
            if (tx_send) begin // If send is high, reset bit counts to default
                tx_counter <= TOTAL_WIDTH - 1;
            end
            else if (cs_n_falling_edge && !CPHA) begin // If CPHA is 0, data is sampled on leading edge
                spi_miso <= tx_data_reg[TOTAL_WIDTH - 1];
                tx_counter <= TOTAL_WIDTH - 2;
            end
            else if ((spi_clk_leading_edge && CPHA) | (spi_clk_trailing_edge && !CPHA)) begin // If CPHA is 1, data is sampled on lagging edge
                tx_counter <= tx_counter - 1'b1;
                spi_miso <= tx_data_reg[tx_counter];
            end
        end
    end

    // SPI MOSI logic
    always @(posedge clk or negedge rst_n)
    begin
        if (!rst_n || spi_cs_n) begin
        rx_data_reg <= {TOTAL_WIDTH{1'b0}};
        rx_header_valid <= 1'b0;
        rx_payload_valid <= 1'b0;
        rx_counter <= TOTAL_WIDTH - 1;
        rx_data <= {TOTAL_WIDTH{1'b0}};
        end
        else begin
            rx_header_valid <= 1'b0;
            rx_payload_valid <= 1'b0;

            if ((spi_clk_leading_edge & !CPHA) | (spi_clk_trailing_edge & CPHA)) begin
                rx_data_reg[rx_counter] <= mosi_reg2;  // Store the received data
                rx_counter <= rx_counter - 1'b1;
                if (rx_counter == TOTAL_WIDTH - HEADER_WIDTH - 1) begin
                    rx_header_valid   <= 1'b1;  // Header received
                end
                else if (rx_counter == 0) begin
                    rx_payload_valid <= 1'b1;   // all the data received
                    rx_data <= {rx_data_reg[TOTAL_WIDTH-1:1], mosi_reg2};    // Store the received data
                end
            end
        end
    end
    
    // Logic to set rx_complete
    assign rx_complete = rx_payload_valid;

    // Logic to set tx_ready
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tx_ready <= 1'b1;
        else if (tx_send)
            tx_ready <= 1'b0;
        else if (rx_complete || spi_cs_n)
            tx_ready <= 1'b1;
    end

endmodule
