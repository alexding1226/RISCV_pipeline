// this is a test bench feeds initial instruction and data
// the processor output is not verified

`timescale 1 ns/10 ps

`define CYCLE 10 // You can modify your clock frequency

`define SDFFILE   "./CHIP_syn.sdf"	// Modify your SDF file name

// new path
`define DMEM_INIT "./Baseline/src/D_mem"

// For different condition (I_mem, TestBed)
`ifdef noHazard
    `define IMEM_INIT "./Baseline/src/I_mem_noHazard"
    `include "./Baseline/src/TestBed_noHazard.v"
`endif
`ifdef hasHazard
	`define IMEM_INIT "./Baseline/src/I_mem_hasHazard"
	`include "./Baseline/src/TestBed_hasHazard.v"
`endif	
`ifdef BrPred
	`define IMEM_INIT "./Extension/BrPred/a10b20c30/I_mem_BrPred"
	`include "./Extension/BrPred/a10b20c30/TestBed_BrPred.v"
`endif
`ifdef compression
	`define IMEM_INIT "./Extension/Compression/DEADxF625/I_mem_compression"
	`include "./Extension/Compression/DEADxF625/TestBed_compression.v"
`endif
`ifdef decompression
	`define IMEM_INIT "./Extension/Compression/DEADxF625/I_mem_decompression"
	`include "./Extension/Compression/DEADxF625/TestBed_compression.v"
`endif
`ifdef L2Cache
	`define IMEM_INIT "./Extension/L2Cache/nb20incre3/I_mem_L2Cache"
	`include "./Extension/L2Cache/nb20incre3/TestBed_L2Cache.v"
`endif

/* 
`define DMEM_INIT "D_mem"

// For different condition (I_mem, TestBed)
`ifdef noHazard
    `define IMEM_INIT "I_mem_noHazard"
    `include "./TestBed_noHazard.v"
`endif
`ifdef hasHazard
	`define IMEM_INIT "I_mem_hasHazard"
	`include "./TestBed_hasHazard.v"
`endif	
`ifdef BrPred
	`define IMEM_INIT "I_mem_BrPred"
	`include "./TestBed_BrPred.v"
`endif
`ifdef compression
	`define IMEM_INIT "I_mem_compression"
	`include "./TestBed_compression.v"
`endif
`ifdef decompression
	`define IMEM_INIT "I_mem_decompression"
	`include "./TestBed_compression.v"
`endif			
 */
 
module Final_tb;

	reg clk;
	reg rst_n;
	
	wire mem_read_D;
	wire mem_write_D;
	wire [31:4] mem_addr_D;
	wire [127:0] mem_wdata_D;
	wire [127:0] mem_rdata_D;
	wire mem_ready_D;

	wire mem_read_I;
	wire mem_write_I;
	wire [31:4] mem_addr_I;
	wire [127:0] mem_wdata_I;
	wire [127:0] mem_rdata_I;
	wire mem_ready_I;

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

	wire [29:0]	DCACHE_addr;
	wire [31:0]	DCACHE_wdata;
	wire 		DCACHE_wen;
	
	wire [7:0] error_num;
	wire [15:0] duration;
	wire finish;	

	integer i_count, d_count;

	// Note the design is connected at testbench, include:
	// 1. CHIP (RISCV + D_cache + I_chache)
	// 2. slow memory for data
	// 3. slow memory for instruction
	
	CHIP chip0 (clk,
				rst_n,
//----------for slow_memD------------	
				mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_wdata_D,
				mem_rdata_D,
				mem_ready_D,
				// L1_read_D,
				// L1_write_D,
				// L1_addr_D,
				// L1_wdata_D,
				// L1_rdata_D,
				// L1_ready_D,
//----------for slow_memI------------
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_wdata_I,
				mem_rdata_I,
				mem_ready_I,
				// L1_read_I,
				// L1_write_I,
				// L1_addr_I,
				// L1_wdata_I,
				// L1_rdata_I,
				// L1_ready_I,
//----------for TestBed--------------				
				DCACHE_addr,
				DCACHE_wdata,
				DCACHE_wen
				);

	/* L2Cache L2Cache0(
		.clk(clk),
		.proc_reset(rst_n),

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
	); */
	
	slow_memory slow_memD(
		.clk        (clk)           ,
		.mem_read   (mem_read_D)    ,
		.mem_write  (mem_write_D)   ,
		.mem_addr   (mem_addr_D)    ,
		.mem_wdata  (mem_wdata_D)   ,
		.mem_rdata  (mem_rdata_D)   ,
		.mem_ready  (mem_ready_D)
	);

	slow_memory slow_memI(
		.clk        (clk)           ,
		.mem_read   (mem_read_I)    ,
		.mem_write  (mem_write_I)   ,
		.mem_addr   (mem_addr_I)    ,
		.mem_wdata  (mem_wdata_I)   ,
		.mem_rdata  (mem_rdata_I)   ,
		.mem_ready  (mem_ready_I)
	);

	TestBed testbed(
		.clk        (clk)           ,
		.rst        (rst_n)         ,
		.addr       (DCACHE_addr)   ,
		.data       (DCACHE_wdata)  ,
		.wen        (DCACHE_wen)    ,
		.error_num  (error_num)     ,
		.duration   (duration)      ,
		.finish     (finish)
	);
	
`ifdef SDF
    initial $sdf_annotate(`SDFFILE, chip0);
`endif

// ============================================
	initial begin
		i_count = 0;
		d_count = 0;
	end
	
	always @(posedge clk) begin
		if (chip0.ICACHE_ren || chip0.ICACHE_wen) begin
			if (chip0.ICACHE_stall == 0) begin
				i_count = i_count + 1;
			end
		end
		if (chip0.DCACHE_ren || chip0.DCACHE_wen) begin
			if (chip0.DCACHE_stall == 0) begin
				d_count = d_count + 1;
			end
		end
	end
// ============================================

// Initialize the data memory
	initial begin
		$display("-----------------------------------------------------\n");
	 	$display("START!!! Simulation Start .....\n");
	 	$display("-----------------------------------------------------\n");
		$readmemb (`DMEM_INIT, slow_memD.mem ); // initialize data in DMEM
		$readmemh (`IMEM_INIT, slow_memI.mem ); // initialize data in IMEM

		// waveform dump
	    // $dumpfile("Final.vcd");
	    // $dumpvars;
	    $fsdbDumpfile("Final_noC.fsdb");			
		$fsdbDumpvars(0,Final_tb,"+mda");
		$fsdbDumpvars;
	
		clk = 0;
		rst_n = 1'b1;
		#(`CYCLE*0.2) rst_n = 1'b0;
		#(`CYCLE*8.5) rst_n = 1'b1;
     
		#(`CYCLE*100000) // calculate clock cycles for all operation (you can modify it)
		// was 10000
		
		
		$display("============================================================================");
		$display("\n           Error!!! There is something wrong with your code ...!          ");
		$display("\n                       The test result is .....FAIL                     \n");
		$display("============================================================================");
		if (testbed.curstate == 2'b0)
			$display("Possible solution: The first answer may not be correct.\n");
		if (testbed.curstate == 2'b1)
			$display("Possible solution: The clock cycles may be too small. Please modify it.\n");
	 	$finish;
	end
		
	always #(`CYCLE*0.5) clk = ~clk;
	
	always@(finish)
	    if(finish) begin
			$display("\n i_count ", i_count);
			$display("\n d_count ", d_count, "\n");
			$display("\n time ", $time, "\n");
		
	       #(`CYCLE) $finish;		   
			
		end
	
endmodule
