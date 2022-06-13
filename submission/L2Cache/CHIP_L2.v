// Top module of your design, you cannot modify this module!!
module CHIP (	clk,
				rst_n,
//----------for slow_memD------------
				mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_wdata_D,
				mem_rdata_D,
				mem_ready_D,
//----------for slow_memI------------
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_wdata_I,
				mem_rdata_I,
				mem_ready_I,
//----------for TestBed--------------				
				DCACHE_addr, 
				DCACHE_wdata,
				DCACHE_wen,
				PC   
			);
input			clk, rst_n;
//--------------------------

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
output	[127:0]	mem_wdata_D;
input	[127:0]	mem_rdata_D;
input			mem_ready_D;
//--------------------------
output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
output	[127:0]	mem_wdata_I;
input	[127:0]	mem_rdata_I;
input			mem_ready_I;
//----------for TestBed--------------
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;
output	[31:0]	PC;
//--------------------------

// wire declaration
wire        ICACHE_ren;
wire        ICACHE_wen;
wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_wdata;
wire        ICACHE_stall;
wire [31:0] ICACHE_rdata;

wire        DCACHE_ren;
wire        DCACHE_wen;
wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_wdata;
wire        DCACHE_stall;
wire [31:0] DCACHE_rdata;
wire [31:0] PC;

// for L2 Cache ==============================================
	wire L1_read_D;
	wire L1_write_D;
	wire [31:4] L1_addr_D;
	wire [127:0] L1_wdata_D;
	wire [127:0] L1_rdata_D;
	wire L1_ready_D;

	wire L1_read_I;
	wire L1_write_I;
	wire [31:4] L1_addr_I;
	wire [127:0] L1_wdata_I;
	wire [127:0] L1_rdata_I;
	wire L1_ready_I;
// ==============================================

//=========================================
	// Note that the overall design of your RISCV includes:
	// 1. pipelined RISCV processor
	// 2. data cache
	// 3. instruction cache


	RISCV_Pipeline i_RISCV(
		// control interface
		.clk            (clk)           , 
		.rst_n          (rst_n)         ,
//----------I cache interface-------		
		.ICACHE_ren     (ICACHE_ren)    ,
		.ICACHE_wen     (ICACHE_wen)    ,
		.ICACHE_addr    (ICACHE_addr)   ,
		.ICACHE_wdata   (ICACHE_wdata)  ,
		.ICACHE_stall   (ICACHE_stall)  ,
		.ICACHE_rdata   (ICACHE_rdata)  ,
//----------D cache interface-------
		.DCACHE_ren     (DCACHE_ren)    ,
		.DCACHE_wen     (DCACHE_wen)    ,
		.DCACHE_addr    (DCACHE_addr)   ,
		.DCACHE_wdata   (DCACHE_wdata)  ,
		.DCACHE_stall   (DCACHE_stall)  ,
		.DCACHE_rdata   (DCACHE_rdata)	,
		//--------------PC-----------------
		.PC(PC)
	);
	

	cache D_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (DCACHE_ren)  ,
        .proc_write (DCACHE_wen)  ,
        .proc_addr  (DCACHE_addr) ,
        .proc_rdata (DCACHE_rdata),
        .proc_wdata (DCACHE_wdata),
        .proc_stall (DCACHE_stall),
        .mem_read   (L1_read_D)  ,
        .mem_write  (L1_write_D) ,
        .mem_addr   (L1_addr_D)  ,
        .mem_wdata  (L1_wdata_D) ,
        .mem_rdata  (L1_rdata_D) ,
        .mem_ready  (L1_ready_D)
	);

	cache_I I_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (ICACHE_ren)  ,
        .proc_write (ICACHE_wen)  ,
        .proc_addr  (ICACHE_addr) ,
        .proc_rdata (ICACHE_rdata),
        .proc_wdata (ICACHE_wdata),
        .proc_stall (ICACHE_stall),
        .mem_read   (L1_read_I)  ,
        .mem_write  (L1_write_I) ,
        .mem_addr   (L1_addr_I)  ,
        .mem_wdata  (L1_wdata_I) ,
        .mem_rdata  (L1_rdata_I) ,
        .mem_ready  (L1_ready_I)
	);

	L2Cache L2Cache0(
		.clk(clk),
		.proc_reset(~rst_n),

		.L1_I_read(L1_read_I),
		.L1_I_write(L1_write_I), // noneed
		.L1_I_addr(L1_addr_I),
		.L1_I_rdata(L1_rdata_I),
		.L1_I_wdata(L1_wdata_I), // noneed
		.L1_I_ready(L1_ready_I),

		.L1_D_read(L1_read_D),
		.L1_D_write(L1_write_D),
		.L1_D_addr(L1_addr_D),
		.L1_D_rdata(L1_rdata_D),
		.L1_D_wdata(L1_wdata_D),
		.L1_D_ready(L1_ready_D),
		
		.mem_I_read(mem_read_I),
		.mem_I_write(mem_write_I), // noneed
		.mem_I_addr(mem_addr_I),
		.mem_I_rdata(mem_rdata_I),
		.mem_I_wdata(mem_wdata_I), // noneed
		.mem_I_ready(mem_ready_I),

		.mem_D_read(mem_read_D),
		.mem_D_write(mem_write_D),
		.mem_D_addr(mem_addr_D),
		.mem_D_rdata(mem_rdata_D),
		.mem_D_wdata(mem_wdata_D),
		.mem_D_ready(mem_ready_D)
	);
endmodule
