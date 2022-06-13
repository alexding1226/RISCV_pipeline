

// 256 words total
`define CONST_block_size 128 // 4 word
`define CONST_block_num 64 // 64 block
`define CONST_associativity 2 // 2way
`define CONST_inst_block_num 32 // 50%

// inclusion policy : inclusive
// try 合成驗證 & Quick sort improvements

// log
// so far : port
// 
/* 
    先試試 3-state，不夠再說 -> 5 state
    instruction prior to data，一次只能一個

    CHIP.v 可以改
    I D 分開?
    > 先做 split，一半一半

    I D 比例：
        hasHazard : 544 / 1869
        L2Cache : 12852 / 46261
        -> 都是 30% 左右需要 data

    // Check:
        // addr 要改，是 word -> 4 word
        // data 寬度也不對
        // rst => rst_n
*/
module L2Cache(
    // modified from HW4
    // 2-way associative cache, 8 block * 4 word
    clk,
    proc_reset,

    L1_I_read,
    L1_I_write, // noneed
    L1_I_addr,
    L1_I_rdata,
    L1_I_wdata, // noneed
    L1_I_ready,

    L1_D_read,
    L1_D_write,
    L1_D_addr,
    L1_D_rdata,
    L1_D_wdata,
    L1_D_ready,
    
    mem_I_read,
    mem_I_write, // noneed
    mem_I_addr,
    mem_I_rdata,
    mem_I_wdata, // noneed
    mem_I_ready,

    mem_D_read,
    mem_D_write,
    mem_D_addr,
    mem_D_rdata,
    mem_D_wdata,
    mem_D_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset; 
    // L1 I_Cache
    input          L1_I_read, L1_I_write;
    input   [27:0] L1_I_addr;
    input   [127:0] L1_I_wdata;
    output /* reg */         L1_I_ready;
    output /* reg */  [127:0] L1_I_rdata;
    // L1 D_Cache
    input          L1_D_read, L1_D_write;
    input   [27:0] L1_D_addr;
    input   [127:0] L1_D_wdata;
    output /* reg */         L1_D_ready;
    output /* reg */  [127:0] L1_D_rdata;
    // I memory interface
    input  [127:0] mem_I_rdata;
    input          mem_I_ready;
    output /* reg */         mem_I_read, mem_I_write;
    output /* reg */ [27:0] mem_I_addr;
    output /* reg */ [127:0] mem_I_wdata;
    // D memory interface
    input  [127:0] mem_D_rdata;
    input          mem_D_ready;
    output /* reg */         mem_D_read, mem_D_write;
    output /* reg */ [27:0] mem_D_addr;
    output /* reg */ [127:0] mem_D_wdata;

    // test
    // assign mem_I_read = L1_I_read;
    // assign mem_I_write = L1_I_write;
    // assign mem_I_addr = L1_I_addr;
    // assign mem_I_wdata = L1_I_wdata;
    // assign L1_I_rdata = mem_I_rdata;
    // assign L1_I_ready = mem_I_ready;

    // assign mem_D_read = L1_D_read;
    // assign mem_D_write = L1_D_write;
    // assign mem_D_addr = L1_D_addr;
    // assign mem_D_wdata = L1_D_wdata;
    // assign L1_D_rdata = mem_D_rdata;
    // assign L1_D_ready = mem_D_ready;

    
// call instances
    L2Cache_I I_Cache(
        .clk(clk),
        .proc_reset(proc_reset),

        .L1_read(L1_I_read),
        .L1_write(L1_I_write),
        .L1_addr(L1_I_addr),
        .L1_rdata(L1_I_rdata),
        .L1_wdata(L1_I_wdata),
        .L1_ready(L1_I_ready),

        .mem_read(mem_I_read),
        .mem_write(mem_I_write),
        .mem_addr(mem_I_addr),
        .mem_rdata(mem_I_rdata),
        .mem_wdata(mem_I_wdata),
        .mem_ready(mem_I_ready)
    );

    L2Cache_D D_Cache(
        .clk(clk),
        .proc_reset(proc_reset),

        .L1_read(L1_D_read),
        .L1_write(L1_D_write),
        .L1_addr(L1_D_addr),
        .L1_rdata(L1_D_rdata),
        .L1_wdata(L1_D_wdata),
        .L1_ready(L1_D_ready),

        .mem_read(mem_D_read),
        .mem_write(mem_D_write),
        .mem_addr(mem_D_addr),
        .mem_rdata(mem_D_rdata),
        .mem_wdata(mem_D_wdata),
        .mem_ready(mem_D_ready)
    );

 /*    
//==== state definition ================================    
    reg [1:0] state_w, state_r;
    reg [1:0] state_D_w, state_D_r;
    localparam S_IDLE = 0;
    localparam S_WRITE_BACK = 1;
    localparam S_ALLOCATION = 2; */

/* //==== wire/reg definition ================================
    
    reg [127:0] data_w [0:(`CONST_block_num / `CONST_associativity)-1][0:`CONST_associativity-1];
    reg [127:0] data_r [0:(`CONST_block_num / `CONST_associativity)-1][0:`CONST_associativity-1];
    reg [25:0] tag_w [0:(`CONST_block_num / `CONST_associativity)-1][0:`CONST_associativity-1]; // 4-word addr 28bit -> 26 (/4)
    reg [25:0] tag_r [0:(`CONST_block_num / `CONST_associativity)-1][0:`CONST_associativity-1];
    reg valid_w [0:(`CONST_block_num / `CONST_associativity)-1][0:`CONST_associativity-1];
    reg valid_r [0:(`CONST_block_num / `CONST_associativity)-1][0:`CONST_associativity-1];
    reg dirty_w [0:(`CONST_block_num / `CONST_associativity)-1][0:`CONST_associativity-1];
    reg dirty_r [0:(`CONST_block_num / `CONST_associativity)-1][0:`CONST_associativity-1];
    reg LRU_w [0:(`CONST_block_num / `CONST_associativity)-1];
    reg LRU_r [0:(`CONST_block_num / `CONST_associativity)-1];
    reg type_w [0:(`CONST_block_num / `CONST_associativity)-1][0:`CONST_associativity-1];
    reg type_r [0:(`CONST_block_num / `CONST_associativity)-1][0:`CONST_associativity-1];

    wire [25:0] I_addr_tag;
    wire [1:0] I_addr_index;
    wire [1:0] I_addr_offset;
    assign addr_tag = L1_I_addr[29:4];
    assign addr_index = L1_I_addr[3:2];
    assign addr_offset = L1_I_addr[1:0];
    
    wire [25:0] D_addr_tag;
    wire [1:0] D_addr_index;
    wire [1:0] D_addr_offset;
    assign addr_tag = L1_I_addr[29:4];
    assign addr_index = L1_I_addr[3:2];
    assign addr_offset = L1_I_addr[1:0];
    
    wire sel_associative; // no use ?

    reg slot_num [0:(`CONST_block_num / `CONST_associativity)-1]; // ?

    integer i,j;
    
 */
 
/* //==== combinational circuit ==============================
    
    always @(*) begin
        // default values
        for (i = 0; i<(`CONST_block_num / `CONST_associativity); i=i+1) begin
            for (j = 0; j<`CONST_associativity; j=j+1) begin
                valid_w [i][j] = valid_r [i][j];
                dirty_w [i][j] = dirty_r [i][j];
                data_w [i][j] = data_r [i][j];
                tag_w [i][j] = tag_r [i][j];
                type_w [i][j] = type_r [i][j];
            end 
            LRU_w [i] = LRU_r [i];
            slot_num[i] = (valid_r[i][0]) ? 
                            (valid_r[i][1] ) ? LRU_r[i] : 1 
                            : 0;
        end
        L1_I_ready = 0;
        L1_I_rdata = 0;
        L1_D_ready = 0;
        L1_D_rdata = 0;
        mem_I_read = 0;
        mem_I_write = 0;
        mem_I_addr = 0;
        mem_I_wdata = 0;
        mem_D_read = 0;
        mem_D_write = 0;
        mem_D_addr = 0;
        mem_D_wdata = 0;
        // modifiable to save area ?
        // default not ready

        state_w = state_r;

        case (state_r)
            S_IDLE:begin
                if(proc_read || proc_write) begin 
                    // identify index, check valid first
                    // then check tag
                    if (valid_r[addr_index][0] && (tag_r[addr_index][0] == addr_tag) ) begin // valid & hit slot 0
                        LRU_w[addr_index] = 1; // set opposite
                        if (proc_read) begin // read
                            case (addr_offset) // *32
                                0: proc_rdata = data_r[addr_index][0][31 : 0];
                                1: proc_rdata = data_r[addr_index][0][63 : 32];
                                2: proc_rdata = data_r[addr_index][0][95 : 64];
                                3: proc_rdata = data_r[addr_index][0][127 : 96];
                            endcase
                        end
                        else begin // write
                            dirty_w[addr_index][0] = 1;
                            case (addr_offset) // *32
                                0: data_w[addr_index][0][31 : 0] = proc_wdata;
                                1: data_w[addr_index][0][63 : 32] = proc_wdata;
                                2: data_w[addr_index][0][95 : 64] = proc_wdata;
                                3: data_w[addr_index][0][127 : 96] = proc_wdata;
                            endcase
                        end
                    end

                    else if (valid_r[addr_index][1] && (tag_r[addr_index][1] == addr_tag) ) begin // valid & hit slot 1
                        LRU_w[addr_index] = 0; // set opposite
                        if (proc_read) begin // read
                            case (addr_offset) // *32
                                0: proc_rdata = data_r[addr_index][1][31 : 0];
                                1: proc_rdata = data_r[addr_index][1][63 : 32];
                                2: proc_rdata = data_r[addr_index][1][95 : 64];
                                3: proc_rdata = data_r[addr_index][1][127 : 96];
                            endcase
                        end
                        else begin // write
                            dirty_w[addr_index][1] = 1;
                            case (addr_offset) // *32
                                0: data_w[addr_index][1][31 : 0] = proc_wdata;
                                1: data_w[addr_index][1][63 : 32] = proc_wdata;
                                2: data_w[addr_index][1][95 : 64] = proc_wdata;
                                3: data_w[addr_index][1][127 : 96] = proc_wdata;
                            endcase
                        end
                    end
                    
                    // structure TBM
                    // no buffer for now
                    // else if ((!valid_r[addr_index][ LRU_r[addr_index] ]) || (!dirty_r[addr_index][ LRU_r[addr_index] ])) begin 
                    else if ((!valid_r[addr_index][ slot_num[addr_index] ]) || (!dirty_r[addr_index][ slot_num[addr_index] ])) begin 
                    // clean or not valid, allocate directly
                        state_w = S_ALLOCATION;
                        proc_stall = 1;
                        mem_read = 1;
                        mem_addr = {addr_tag, addr_index};
                    end
                    else begin // write back existing data
                        state_w = S_WRITE_BACK;
                        proc_stall = 1;
                        mem_write = 1;
                        // mem_addr = {tag_r[addr_index][ LRU_r[addr_index] ], addr_index};
                        mem_addr = {tag_r[addr_index][ slot_num[addr_index] ], addr_index};
                        // mem_wdata = data_r[addr_index][ LRU_r[addr_index] ];
                        mem_wdata = data_r[addr_index][ slot_num[addr_index] ];
                    end

                    //fetch with write allocation(fetch when write miss)
                    
                end
                
            end

            S_WRITE_BACK: begin
                proc_stall = 1;
                mem_write = 1;
                mem_addr = {tag_r[addr_index][ slot_num[addr_index] ], addr_index};
                mem_wdata = data_r[addr_index][ slot_num[addr_index] ];
                // how modify, avoid duplicate

                if (mem_ready) begin 
                    state_w = S_ALLOCATION;
                    proc_stall = 1; 
                    mem_read = 1;
                    mem_addr = {addr_tag, addr_index};
                end
            end

            S_ALLOCATION: begin
                proc_stall = 1;
                mem_read = 1;
                mem_addr = {addr_tag, addr_index};
                // how modify, avoid duplicate
                
                if (mem_ready) begin // half cycle left for CPU ?    
                    data_w[addr_index][ slot_num[addr_index] ] = mem_rdata;   
                    valid_w[addr_index][ slot_num[addr_index] ] = 1;
                    dirty_w[addr_index][ slot_num[addr_index] ] = 0;
                    tag_w[addr_index][ slot_num[addr_index] ] = addr_tag;
                    state_w = S_IDLE;
                    
                    proc_stall = 0;
                    // LRU_w[addr_index] = ~LRU_r[addr_index];
                    LRU_w[addr_index] = ~slot_num[addr_index];
                    if (proc_read) begin
                        case (addr_offset) // *32
                            0: proc_rdata = mem_rdata[31 : 0];
                            1: proc_rdata = mem_rdata[63 : 32];
                            2: proc_rdata = mem_rdata[95 : 64];
                            3: proc_rdata = mem_rdata[127 : 96];
                        endcase
                    end
                    else begin // write
                        dirty_w[addr_index][ slot_num[addr_index] ] = 1;
                        case (addr_offset) // *32
                            0: data_w[addr_index][ slot_num[addr_index] ][31 : 0] = proc_wdata;
                            1: data_w[addr_index][ slot_num[addr_index] ][63 : 32] = proc_wdata;
                            2: data_w[addr_index][ slot_num[addr_index] ][95 : 64] = proc_wdata;
                            3: data_w[addr_index][ slot_num[addr_index] ][127 : 96] = proc_wdata;
                        endcase
                    end

                end
            end

        endcase
    end
     */
/* //==== sequential circuit =================================
    always@( posedge clk ) begin
        if( proc_reset ) begin
            for (i = 0; i<(`CONST_block_num / `CONST_associativity); i=i+1) begin
                for (j = 0; j<`CONST_associativity; j=j+1) begin
                    valid_r[i][j] <= 0;
                    dirty_r[i][j] <= 0;
                end
                LRU_r[i] <= 0;
            end
            
            state_r <= S_IDLE;
        end

        else begin
            for (i = 0; i<(`CONST_block_num / `CONST_associativity); i=i+1) begin
                for (j = 0; j<`CONST_associativity; j=j+1) begin
                    valid_r [i][j] <= valid_w [i][j];
                    dirty_r [i][j] <= dirty_w [i][j];
                    data_r [i][j] <= data_w [i][j];
                    tag_r [i][j] <= tag_w [i][j];
                    type_r [i][j] <= type_w [i][j];
                end      
                LRU_r[i] <= LRU_w[i];
            end

            state_r <= state_w;
        end
    end */

endmodule

module L2Cache_D
    // individual split cache
    // default 2-way associative cache, 32 block * 4 word
    
    // block size 改大一點?
    // sequential 要確定一下
        // 現在 ready & rdata 會用 reg 擋

    // 先做個能跑的東西出來，之後再修

    # (
        parameter para_block_size = 128, // 4 word 
        parameter para_block_num = 16/* 32 */, // 32 blocks
        parameter para_associativity = 2 // 2way
    )
    (
        clk,
        proc_reset,

        L1_read,
        L1_write,
        L1_addr,
        L1_rdata,
        L1_wdata,
        L1_ready,
        
        mem_read,
        mem_write,
        mem_addr,
        mem_rdata,
        mem_wdata,
        mem_ready
    );
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset; // active low ?
    // L1 Cache
    input          L1_read, L1_write;
    input   [27:0] L1_addr;
    input   [127:0] L1_wdata;
    output reg         L1_ready;
    output reg  [127:0] L1_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output reg         mem_read, mem_write;
    output reg [27:0] mem_addr;
    output reg [127:0] mem_wdata;
    // read & write 是從上一級的角度

    reg         L1_ready_w, L1_ready_r;
    reg  [127:0] L1_rdata_w, L1_rdata_r;
//==== state definition ================================    
    reg [1:0] state_w, state_r;
    localparam S_IDLE = 0;
    localparam S_WRITE_BACK = 1;
    localparam S_ALLOCATION = 2;
    localparam S_STORAGE = 3;

//==== wire/reg definition ================================
    
    reg [127:0] data_w [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg [127:0] data_r [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg [27-log2(para_block_num / para_associativity):0] tag_w [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg [27-log2(para_block_num / para_associativity):0] tag_r [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg valid_w [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg valid_r [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg dirty_w [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg dirty_r [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg [log2(para_associativity)-1:0] LRU_w [0:(para_block_num / para_associativity)-1];
    reg [log2(para_associativity)-1:0] LRU_r [0:(para_block_num / para_associativity)-1];
    reg type_w [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg type_r [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    
    function [31:0] log2;
        input [31:0] value;
        integer i;
        reg [31:0] j;
        begin
            j = value - 1;
            log2 = 0;
            for (i = 0; i < 31; i = i + 1)
                if (j[i]) log2 = i+1;
        end
    endfunction
    
    // 4-word addr incoming [27:0]
    // 4-word addr output
    wire [27-log2(para_block_num / para_associativity):0] addr_tag;
    wire [log2(para_block_num / para_associativity)-1:0] addr_index;
    assign addr_tag = L1_addr[27:log2(para_block_num / para_associativity)]; // mem
    assign addr_index = L1_addr[log2(para_block_num / para_associativity)-1:0]; // mem
    
    // 4-word storage, no longer need index
    // wire [1:0] addr_offset;
    // assign addr_offset = L1_addr[1:0]; 
    
    reg [log2(para_associativity)-1:0] slot_num [0:(para_block_num / para_associativity)-1]; // 要被蓋掉的?

    integer i,j;

    reg [127:0] mem_rdata_r;
    
//==== combinational circuit ==============================
    
    always @(*) begin
        // default values
        for (i = 0; i<(para_block_num / para_associativity); i=i+1) begin
            for (j = 0; j<para_associativity; j=j+1) begin
                valid_w [i][j] = valid_r [i][j];
                dirty_w [i][j] = dirty_r [i][j];
                data_w [i][j] = data_r [i][j];
                tag_w [i][j] = tag_r [i][j];
                type_w [i][j] = type_r [i][j];
            end 
            LRU_w [i] = LRU_r [i];
            slot_num[i] = (valid_r[i][0]) ? 
                            (valid_r[i][1] ) ? LRU_r[i] : 1 
                            : 0;
        end
        L1_ready = L1_ready_r;
        L1_rdata = L1_rdata_r;
        L1_ready_w = 0/* L1_ready_r */;
        L1_rdata_w = L1_rdata_r;

        mem_read = 0;
        mem_write = 0;
        mem_addr = 0;
        mem_wdata = 0;
        // modifiable to save area ?
        // default not ready

        state_w = state_r;

        case (state_r)
            S_IDLE:begin
                // expecting 同一個 cycle 就可以拿到 data
                // 但先不想改 L1 cache -> 用個 reg 擋

                if(L1_read || L1_write) begin 
                    // identify index, check valid first
                    // then check tag
                    if (valid_r[addr_index][0] && (tag_r[addr_index][0] == addr_tag) ) begin // valid & hit slot 0
                        LRU_w[addr_index] = 1; // set opposite
                        L1_ready_w = 1;
                        if (L1_read) begin // read
                            // case (addr_offset) // *32
                            //     0: L1_rdata_w = data_r[addr_index][0][31 : 0];
                            //     1: L1_rdata_w = data_r[addr_index][0][63 : 32];
                            //     2: L1_rdata_w = data_r[addr_index][0][95 : 64];
                            //     3: L1_rdata_w = data_r[addr_index][0][127 : 96];
                            // endcase
                            L1_rdata_w = data_r[addr_index][0];
                        end
                        else begin // write
                            dirty_w[addr_index][0] = 1;
                            // case (addr_offset) // *32
                            //     0: data_w[addr_index][0][31 : 0] = L1_wdata;
                            //     1: data_w[addr_index][0][63 : 32] = L1_wdata;
                            //     2: data_w[addr_index][0][95 : 64] = L1_wdata;
                            //     3: data_w[addr_index][0][127 : 96] = L1_wdata;
                            // endcase
                            data_w[addr_index][0] = L1_wdata;
                        end
                    end

                    else if (valid_r[addr_index][1] && (tag_r[addr_index][1] == addr_tag) ) begin // valid & hit slot 1
                        LRU_w[addr_index] = 0; // set opposite
                        L1_ready_w = 1;
                        if (L1_read) begin // read
                            // case (addr_offset) // *32
                            //     0: L1_rdata_w = data_r[addr_index][1][31 : 0];
                            //     1: L1_rdata_w = data_r[addr_index][1][63 : 32];
                            //     2: L1_rdata_w = data_r[addr_index][1][95 : 64];
                            //     3: L1_rdata_w = data_r[addr_index][1][127 : 96];
                            // endcase
                            L1_rdata_w = data_r[addr_index][1];
                        end
                        else begin // write
                            dirty_w[addr_index][1] = 1;
                            // case (addr_offset) // *32
                            //     0: data_w[addr_index][1][31 : 0] = L1_wdata;
                            //     1: data_w[addr_index][1][63 : 32] = L1_wdata;
                            //     2: data_w[addr_index][1][95 : 64] = L1_wdata;
                            //     3: data_w[addr_index][1][127 : 96] = L1_wdata;
                            // endcase
                            data_w[addr_index][1] = L1_wdata;
                        end
                    end
                    
                    // structure TBM
                    // no buffer for now
                    // else if ((!valid_r[addr_index][ LRU_r[addr_index] ]) || (!dirty_r[addr_index][ LRU_r[addr_index] ])) begin 
                    else if ((!valid_r[addr_index][ slot_num[addr_index] ]) || (!dirty_r[addr_index][ slot_num[addr_index] ])) begin 
                    // clean or not valid, allocate directly
                        state_w = S_ALLOCATION;
                        L1_ready_w = 0;

                        mem_read = 1;
                        mem_addr = {addr_tag, addr_index};
                    end
                    else begin // write back existing data
                        state_w = S_WRITE_BACK;
                        L1_ready_w = 0;

                        // mem_write = 1;
                        // // mem_addr = {tag_r[addr_index][ LRU_r[addr_index] ], addr_index};
                        // mem_addr = {tag_r[addr_index][ slot_num[addr_index] ], addr_index};
                        // // mem_wdata = data_r[addr_index][ LRU_r[addr_index] ];
                        // mem_wdata = data_r[addr_index][ slot_num[addr_index] ];
                    end

                    //fetch with write allocation(fetch when write miss)
                    
                end
                
            end

            S_WRITE_BACK: begin
                L1_ready_w = 0;

                mem_write = 1;
                mem_addr = {tag_r[addr_index][ slot_num[addr_index] ], addr_index};
                mem_wdata = data_r[addr_index][ slot_num[addr_index] ];
                // how modify, avoid duplicate

                if (mem_ready) begin 
                    state_w = S_ALLOCATION;
                    L1_ready_w = 0;

                    mem_read = 1;
                    mem_addr = {addr_tag, addr_index};
                end
            end

            S_ALLOCATION: begin
                L1_ready_w = 0;

                mem_read = 1;
                mem_addr = {addr_tag, addr_index};
                // how modify, avoid duplicate
                
                if (mem_ready) begin // half cycle left for CPU ? 
                    state_w = S_STORAGE;

                    /* data_w[addr_index][ slot_num[addr_index] ] = mem_rdata;   
                    valid_w[addr_index][ slot_num[addr_index] ] = 1;
                    dirty_w[addr_index][ slot_num[addr_index] ] = 0;
                    tag_w[addr_index][ slot_num[addr_index] ] = addr_tag;
                    state_w = S_IDLE;

                    L1_ready_w = 1;
                    
                    // proc_stall = 0;
                    // LRU_w[addr_index] = ~LRU_r[addr_index];
                    LRU_w[addr_index] = ~slot_num[addr_index]; */

                    /* if (L1_read) begin
                        // case (addr_offset) // *32
                        //     0: L1_rdata_w = mem_rdata[31 : 0];
                        //     1: L1_rdata_w = mem_rdata[63 : 32];
                        //     2: L1_rdata_w = mem_rdata[95 : 64];
                        //     3: L1_rdata_w = mem_rdata[127 : 96];
                        // endcase
                        L1_rdata_w = mem_rdata;
                    end
                    else begin // write
                        dirty_w[addr_index][ slot_num[addr_index] ] = 1;
                        // case (addr_offset) // *32
                        //     0: data_w[addr_index][ slot_num[addr_index] ][31 : 0] = L1_wdata;
                        //     1: data_w[addr_index][ slot_num[addr_index] ][63 : 32] = L1_wdata;
                        //     2: data_w[addr_index][ slot_num[addr_index] ][95 : 64] = L1_wdata;
                        //     3: data_w[addr_index][ slot_num[addr_index] ][127 : 96] = L1_wdata;
                        // endcase
                        data_w[addr_index][ slot_num[addr_index] ] = L1_wdata;
                    end */

                end
            end

            S_STORAGE: begin
                data_w[addr_index][ slot_num[addr_index] ] = mem_rdata_r;   
                valid_w[addr_index][ slot_num[addr_index] ] = 1;
                dirty_w[addr_index][ slot_num[addr_index] ] = 0;
                tag_w[addr_index][ slot_num[addr_index] ] = addr_tag;
                
                state_w = S_IDLE;
                L1_ready_w = 1;

                LRU_w[addr_index] = ~slot_num[addr_index];

                if (L1_read) begin
                    L1_rdata_w = mem_rdata_r;
                end
                else begin // write
                    dirty_w[addr_index][ slot_num[addr_index] ] = 1;
                    
                    data_w[addr_index][ slot_num[addr_index] ] = L1_wdata;
                end
            end

        endcase
    end
    
//==== sequential circuit =================================
    always@( posedge clk ) begin
        if( proc_reset ) begin
            for (i = 0; i<(para_block_num / para_associativity); i=i+1) begin
                for (j = 0; j<para_associativity; j=j+1) begin
                    valid_r[i][j] <= 0;
                    dirty_r[i][j] <= 0;
                end
                LRU_r[i] <= 0;
            end
            
            state_r <= S_IDLE;
            L1_rdata_r <= 0;
            L1_ready_r <= 0;
        end

        else begin
            for (i = 0; i<(para_block_num / para_associativity); i=i+1) begin
                for (j = 0; j<para_associativity; j=j+1) begin
                    valid_r [i][j] <= valid_w [i][j];
                    dirty_r [i][j] <= dirty_w [i][j];
                    data_r [i][j] <= data_w [i][j];
                    tag_r [i][j] <= tag_w [i][j];
                    type_r [i][j] <= type_w [i][j];
                end      
                LRU_r[i] <= LRU_w[i];
            end

            state_r <= state_w;
            L1_rdata_r <= L1_rdata_w;
            L1_ready_r <= L1_ready_w;
        end
    end

    always @(posedge clk) begin
        if (proc_reset)begin
            mem_rdata_r <= 0;
        end
        mem_rdata_r <= mem_rdata;
    end

endmodule

module L2Cache_I
    // individual split cache
    // default 2-way associative cache, 32 block * 4 word
    
    // block size 改大一點?
    // sequential 要確定一下
        // 現在 ready & rdata 會用 reg 擋

    // 先做個能跑的東西出來，之後再修

    # (
        parameter para_block_size = 128, // 4 word 
        parameter para_block_num = 16/* 32 */, // 32 blocks
        parameter para_associativity = 2 // 2way
    )
    (
        clk,
        proc_reset,

        L1_read,
        L1_write,
        L1_addr,
        L1_rdata,
        L1_wdata,
        L1_ready,
        
        mem_read,
        mem_write,
        mem_addr,
        mem_rdata,
        mem_wdata,
        mem_ready
    );
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset; // active low ?
    // L1 Cache
    input          L1_read, L1_write;
    input   [27:0] L1_addr;
    input   [127:0] L1_wdata;
    output reg         L1_ready;
    output reg  [127:0] L1_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output reg         mem_read, mem_write;
    output reg [27:0] mem_addr;
    output reg [127:0] mem_wdata;
    // read & write 是從上一級的角度

    reg         L1_ready_w, L1_ready_r;
    reg  [127:0] L1_rdata_w, L1_rdata_r;
//==== state definition ================================    
    reg [1:0] state_w, state_r;
    localparam S_IDLE = 0;
    localparam S_WRITE_BACK = 1;
    localparam S_ALLOCATION = 2;
    localparam S_STORAGE = 3;

//==== wire/reg definition ================================
    
    reg [127:0] data_w [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg [127:0] data_r [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg [27-log2(para_block_num / para_associativity):0] tag_w [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg [27-log2(para_block_num / para_associativity):0] tag_r [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg valid_w [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg valid_r [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg dirty_w [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg dirty_r [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg [log2(para_associativity)-1:0] LRU_w [0:(para_block_num / para_associativity)-1];
    reg [log2(para_associativity)-1:0] LRU_r [0:(para_block_num / para_associativity)-1];
    reg type_w [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    reg type_r [0:(para_block_num / para_associativity)-1][0:para_associativity-1];
    
    function [31:0] log2;
        input [31:0] value;
        integer i;
        reg [31:0] j;
        begin
            j = value - 1;
            log2 = 0;
            for (i = 0; i < 31; i = i + 1)
                if (j[i]) log2 = i+1;
        end
    endfunction
    
    // 4-word addr incoming [27:0]
    // 4-word addr output
    wire [27-log2(para_block_num / para_associativity):0] addr_tag;
    wire [log2(para_block_num / para_associativity)-1:0] addr_index;
    assign addr_tag = L1_addr[27:log2(para_block_num / para_associativity)]; // mem
    assign addr_index = L1_addr[log2(para_block_num / para_associativity)-1:0]; // mem
    
    // 4-word storage, no longer need index
    // wire [1:0] addr_offset;
    // assign addr_offset = L1_addr[1:0]; 
    
    reg [log2(para_associativity)-1:0] slot_num [0:(para_block_num / para_associativity)-1]; // 要被蓋掉的?

    integer i,j;

    reg [127:0] mem_rdata_r;
    
//==== combinational circuit ==============================
    
    always @(*) begin
        // default values
        for (i = 0; i<(para_block_num / para_associativity); i=i+1) begin
            for (j = 0; j<para_associativity; j=j+1) begin
                valid_w [i][j] = valid_r [i][j];
                dirty_w [i][j] = dirty_r [i][j];
                data_w [i][j] = data_r [i][j];
                tag_w [i][j] = tag_r [i][j];
                type_w [i][j] = type_r [i][j];
            end 
            LRU_w [i] = LRU_r [i];
            slot_num[i] = (valid_r[i][0]) ? 
                            (valid_r[i][1] ) ? LRU_r[i] : 1 
                            : 0;
        end
        L1_ready = L1_ready_r;
        L1_rdata = L1_rdata_r;
        L1_ready_w = 0/* L1_ready_r */;
        L1_rdata_w = L1_rdata_r;

        mem_read = 0;
        mem_write = 0;
        mem_addr = 0;
        mem_wdata = 0;
        // modifiable to save area ?
        // default not ready

        state_w = state_r;

        case (state_r)
            S_IDLE:begin
                // expecting 同一個 cycle 就可以拿到 data
                // 但先不想改 L1 cache -> 用個 reg 擋

                if(L1_read || L1_write) begin 
                    // identify index, check valid first
                    // then check tag
                    if (valid_r[addr_index][0] && (tag_r[addr_index][0] == addr_tag) ) begin // valid & hit slot 0
                        LRU_w[addr_index] = 1; // set opposite
                        L1_ready_w = 1;
                        
                        // read only
                        // if (L1_read) begin // read
                            // case (addr_offset) // *32
                            //     0: L1_rdata_w = data_r[addr_index][0][31 : 0];
                            //     1: L1_rdata_w = data_r[addr_index][0][63 : 32];
                            //     2: L1_rdata_w = data_r[addr_index][0][95 : 64];
                            //     3: L1_rdata_w = data_r[addr_index][0][127 : 96];
                            // endcase
                            L1_rdata_w = data_r[addr_index][0];
                        // end
                        /* else begin // write
                            dirty_w[addr_index][0] = 1;
                            // case (addr_offset) // *32
                            //     0: data_w[addr_index][0][31 : 0] = L1_wdata;
                            //     1: data_w[addr_index][0][63 : 32] = L1_wdata;
                            //     2: data_w[addr_index][0][95 : 64] = L1_wdata;
                            //     3: data_w[addr_index][0][127 : 96] = L1_wdata;
                            // endcase
                            data_w[addr_index][0] = L1_wdata;
                        end */
                    end

                    else if (valid_r[addr_index][1] && (tag_r[addr_index][1] == addr_tag) ) begin // valid & hit slot 1
                        LRU_w[addr_index] = 0; // set opposite
                        L1_ready_w = 1;
                        
                        // read only
                        // if (L1_read) begin // read
                            // case (addr_offset) // *32
                            //     0: L1_rdata_w = data_r[addr_index][1][31 : 0];
                            //     1: L1_rdata_w = data_r[addr_index][1][63 : 32];
                            //     2: L1_rdata_w = data_r[addr_index][1][95 : 64];
                            //     3: L1_rdata_w = data_r[addr_index][1][127 : 96];
                            // endcase
                            L1_rdata_w = data_r[addr_index][1];
                        // end
                        /* else begin // write
                            dirty_w[addr_index][1] = 1;
                            // case (addr_offset) // *32
                            //     0: data_w[addr_index][1][31 : 0] = L1_wdata;
                            //     1: data_w[addr_index][1][63 : 32] = L1_wdata;
                            //     2: data_w[addr_index][1][95 : 64] = L1_wdata;
                            //     3: data_w[addr_index][1][127 : 96] = L1_wdata;
                            // endcase
                            data_w[addr_index][1] = L1_wdata;
                        end */
                    end
                    
                    // structure TBM
                    // no buffer for now
                    // else if ((!valid_r[addr_index][ LRU_r[addr_index] ]) || (!dirty_r[addr_index][ LRU_r[addr_index] ])) begin 

                    // else if ((!valid_r[addr_index][ slot_num[addr_index] ]) || (!dirty_r[addr_index][ slot_num[addr_index] ])) begin 
                    // clean or not valid, allocate directly
                    else begin
                        state_w = S_ALLOCATION;
                        L1_ready_w = 0;

                        mem_read = 1;
                        mem_addr = {addr_tag, addr_index};
                    end
                    /* else begin // write back existing data
                        state_w = S_WRITE_BACK;
                        L1_ready_w = 0;

                        // mem_write = 1;
                        // // mem_addr = {tag_r[addr_index][ LRU_r[addr_index] ], addr_index};
                        // mem_addr = {tag_r[addr_index][ slot_num[addr_index] ], addr_index};
                        // // mem_wdata = data_r[addr_index][ LRU_r[addr_index] ];
                        // mem_wdata = data_r[addr_index][ slot_num[addr_index] ];
                    end */

                    //fetch with write allocation(fetch when write miss)
                    
                end
                
            end

            /* S_WRITE_BACK: begin
                L1_ready_w = 0;

                mem_write = 1;
                mem_addr = {tag_r[addr_index][ slot_num[addr_index] ], addr_index};
                mem_wdata = data_r[addr_index][ slot_num[addr_index] ];
                // how modify, avoid duplicate

                if (mem_ready) begin 
                    state_w = S_ALLOCATION;
                    L1_ready_w = 0;

                    mem_read = 1;
                    mem_addr = {addr_tag, addr_index};
                end
            end */

            S_ALLOCATION: begin
                L1_ready_w = 0;

                mem_read = 1;
                mem_addr = {addr_tag, addr_index};
                // how modify, avoid duplicate
                
                if (mem_ready) begin // half cycle left for CPU ? 
                    state_w = S_STORAGE;

                    /* data_w[addr_index][ slot_num[addr_index] ] = mem_rdata;   
                    valid_w[addr_index][ slot_num[addr_index] ] = 1;
                    dirty_w[addr_index][ slot_num[addr_index] ] = 0;
                    tag_w[addr_index][ slot_num[addr_index] ] = addr_tag;
                    state_w = S_IDLE;

                    L1_ready_w = 1;
                    
                    // proc_stall = 0;
                    // LRU_w[addr_index] = ~LRU_r[addr_index];
                    LRU_w[addr_index] = ~slot_num[addr_index]; */

                    /* if (L1_read) begin
                        // case (addr_offset) // *32
                        //     0: L1_rdata_w = mem_rdata[31 : 0];
                        //     1: L1_rdata_w = mem_rdata[63 : 32];
                        //     2: L1_rdata_w = mem_rdata[95 : 64];
                        //     3: L1_rdata_w = mem_rdata[127 : 96];
                        // endcase
                        L1_rdata_w = mem_rdata;
                    end
                    else begin // write
                        dirty_w[addr_index][ slot_num[addr_index] ] = 1;
                        // case (addr_offset) // *32
                        //     0: data_w[addr_index][ slot_num[addr_index] ][31 : 0] = L1_wdata;
                        //     1: data_w[addr_index][ slot_num[addr_index] ][63 : 32] = L1_wdata;
                        //     2: data_w[addr_index][ slot_num[addr_index] ][95 : 64] = L1_wdata;
                        //     3: data_w[addr_index][ slot_num[addr_index] ][127 : 96] = L1_wdata;
                        // endcase
                        data_w[addr_index][ slot_num[addr_index] ] = L1_wdata;
                    end */

                end
            end

            S_STORAGE: begin
                data_w[addr_index][ slot_num[addr_index] ] = mem_rdata_r;   
                valid_w[addr_index][ slot_num[addr_index] ] = 1;
                dirty_w[addr_index][ slot_num[addr_index] ] = 0;
                tag_w[addr_index][ slot_num[addr_index] ] = addr_tag;
                
                state_w = S_IDLE;
                L1_ready_w = 1;

                LRU_w[addr_index] = ~slot_num[addr_index];

                // if (L1_read) begin
                    L1_rdata_w = mem_rdata_r;
                // end
                // else begin // write
                //     dirty_w[addr_index][ slot_num[addr_index] ] = 1;
                    
                //     data_w[addr_index][ slot_num[addr_index] ] = L1_wdata;
                // end
            end

        endcase
    end
    
//==== sequential circuit =================================
    always@( posedge clk ) begin
        if( proc_reset ) begin
            for (i = 0; i<(para_block_num / para_associativity); i=i+1) begin
                for (j = 0; j<para_associativity; j=j+1) begin
                    valid_r[i][j] <= 0;
                    dirty_r[i][j] <= 0;
                end
                LRU_r[i] <= 0;
            end
            
            state_r <= S_IDLE;
            L1_rdata_r <= 0;
            L1_ready_r <= 0;
        end

        else begin
            for (i = 0; i<(para_block_num / para_associativity); i=i+1) begin
                for (j = 0; j<para_associativity; j=j+1) begin
                    valid_r [i][j] <= valid_w [i][j];
                    dirty_r [i][j] <= dirty_w [i][j];
                    data_r [i][j] <= data_w [i][j];
                    tag_r [i][j] <= tag_w [i][j];
                    type_r [i][j] <= type_w [i][j];
                end      
                LRU_r[i] <= LRU_w[i];
            end

            state_r <= state_w;
            L1_rdata_r <= L1_rdata_w;
            L1_ready_r <= L1_ready_w;
        end
    end

    always @(posedge clk) begin
        if (proc_reset)begin
            mem_rdata_r <= 0;
        end
        mem_rdata_r <= mem_rdata;
    end

endmodule