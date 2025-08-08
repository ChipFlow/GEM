
module \$__RAMGEM_SYNC_ ( PORT_R_CLK, PORT_R_ADDR, PORT_R_RD_DATA, PORT_W_CLK, PORT_W_WR_EN, PORT_W_ADDR, PORT_W_WR_DATA );
   input PORT_R_CLK;
   input [12:0] PORT_R_ADDR;
   output reg [31:0] PORT_R_RD_DATA;

   input PORT_W_CLK;
   input [31:0] PORT_W_WR_EN;
   input [12:0] PORT_W_ADDR;
   input [31:0] PORT_W_WR_DATA;

   reg [31:0]   mem[0:(1<<13) - 1];

   //always @(posedge PORT_R_CLK)


   reg [31:0] i;
   always @(posedge PORT_W_CLK) begin
     PORT_R_RD_DATA <= mem[PORT_R_ADDR];
     for (i=0; i<32; i=i+1)
       if (PORT_W_WR_EN[i]) mem[PORT_W_ADDR][i] <= PORT_W_WR_DATA[i];
   end
endmodule

/*
module \$__RAMGEM_ASYNC_ ( PORT_R_ADDR, PORT_R_RD_DATA, PORT_W_CLK, PORT_W_WR_EN, PORT_W_ADDR, PORT_W_WR_DATA );
   input [12:0] PORT_R_ADDR;
   output [31:0] PORT_R_RD_DATA;

   input PORT_W_CLK;
   input [31:0] PORT_W_WR_EN;
   input [12:0] PORT_W_ADDR;
   input [31:0] PORT_W_WR_DATA;

   reg [31:0]   mem[0:(1<<13) - 1];

   assign PORT_R_RD_DATA = mem[PORT_R_ADDR];

   reg [31:0] i;
   always @(posedge PORT_W_CLK)
     for (i=0; i<32; i=i+1)
       if (PORT_W_WR_EN[i]) mem[PORT_W_ADDR][i] <= PORT_W_WR_DATA[i];
endmodule
*/
