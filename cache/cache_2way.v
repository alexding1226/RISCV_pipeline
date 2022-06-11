module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
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
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output reg         proc_stall;
    output reg  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output reg         mem_read, mem_write;
    output reg [27:0] mem_addr;
    output reg [127:0] mem_wdata;
    
//==== state definition ================================    
    reg [1:0] state_w, state_r;
    localparam S_IDLE = 0;
    localparam S_WRITE_BACK = 1;
    localparam S_ALLOCATION = 2;

//==== wire/reg definition ================================
    
    reg [127:0] data_w [0:3][0:1];
    reg [127:0] data_r [0:3][0:1];
    reg [25:0] tag_w [0:3][0:1]; // 4-word addr 28bit -> 26 (/4)
    reg [25:0] tag_r [0:3][0:1];
    reg valid_w [0:3][0:1];
    reg valid_r [0:3][0:1];
    reg dirty_w [0:3][0:1];
    reg dirty_r [0:3][0:1];
    reg LRU_w [0:3];
    reg LRU_r [0:3];
    // why can't I declare them in the same line ?

    wire [25:0] addr_tag;
    wire [1:0] addr_index;
    wire [1:0] addr_offset;
    wire sel_associative;
    assign addr_tag = proc_addr[29:4];
    assign addr_index = proc_addr[3:2];
    assign addr_offset = proc_addr[1:0];

    reg slot_num [0:3];

    integer i,j;

    reg [127:0] mem_rdata_r;
    reg mem_ready_r;
    
//==== combinational circuit ==============================
    
    always @(*) begin
        // default values
        for (i = 0; i<4; i=i+1) begin
            for (j = 0; j<2; j=j+1) begin
                valid_w [i][j] = valid_r [i][j];
                dirty_w [i][j] = dirty_r [i][j];
                data_w [i][j] = data_r [i][j];
                tag_w [i][j] = tag_r [i][j];
            end 
            LRU_w [i] = LRU_r [i];
            slot_num[i] = (valid_r[i][0]) ? 
                            (valid_r[i][1] ) ? LRU_r[i] : 1 
                            : 0;
        end
        proc_stall = 0;
        proc_rdata = 0;
        mem_read = 0;
        mem_write = 0;
        mem_addr = 0;
        mem_wdata = 0;
        // modifiable to save area ?

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

                if (mem_ready_r) begin 
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
                
                if (mem_ready_r) begin // half cycle left for CPU ?    
                    data_w[addr_index][ slot_num[addr_index] ] = mem_rdata_r;   
                    valid_w[addr_index][ slot_num[addr_index] ] = 1;
                    dirty_w[addr_index][ slot_num[addr_index] ] = 0;
                    tag_w[addr_index][ slot_num[addr_index] ] = addr_tag;
                    state_w = S_IDLE;
                    
                    //proc_stall = 0;
                    // LRU_w[addr_index] = ~LRU_r[addr_index];
                    LRU_w[addr_index] = ~slot_num[addr_index];
                    if (proc_read) begin
                        case (addr_offset) // *32
                            0: proc_rdata = mem_rdata_r[31 : 0];
                            1: proc_rdata = mem_rdata_r[63 : 32];
                            2: proc_rdata = mem_rdata_r[95 : 64];
                            3: proc_rdata = mem_rdata_r[127 : 96];
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
    
//==== sequential circuit =================================
    always@( posedge clk ) begin
        if( proc_reset ) begin
            for (i = 0; i<4; i=i+1) begin
                for (j = 0; j<2; j=j+1) begin
                    valid_r[i][j] <= 0;
                    dirty_r[i][j] <= 0;
                end
                LRU_r[i] <= 0;
            end
            
            state_r <= S_IDLE;
        end

        else begin
            for (i = 0; i<4; i=i+1) begin
                for (j = 0; j<2; j=j+1) begin
                    valid_r [i][j] <= valid_w [i][j];
                    dirty_r [i][j] <= dirty_w [i][j];
                    data_r [i][j] <= data_w [i][j];
                    tag_r [i][j] <= tag_w [i][j];
                end      
                LRU_r[i] <= LRU_w[i];
            end

            state_r <= state_w;
        end
    end
    always @(posedge clk) begin
        if (proc_reset)begin
            mem_rdata_r <= 0;
            mem_rdata_r <= 0;
        end
        mem_rdata_r <= mem_rdata;
        mem_ready_r <= mem_ready;
    end

endmodule


module cache_I(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
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
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output reg         proc_stall;
    output reg  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output reg         mem_read, mem_write;
    output reg [27:0] mem_addr;
    output reg [127:0] mem_wdata;
    
//==== state definition ================================    
    reg [1:0] state_w, state_r;
    localparam S_IDLE = 0;
    localparam S_WRITE_BACK = 1;
    localparam S_ALLOCATION = 2;

//==== wire/reg definition ================================
    
    reg [127:0] data_w [0:3][0:1];
    reg [127:0] data_r [0:3][0:1];
    reg [25:0] tag_w [0:3][0:1]; // 4-word addr 28bit -> 26 (/4)
    reg [25:0] tag_r [0:3][0:1];
    reg valid_w [0:3][0:1];
    reg valid_r [0:3][0:1];
    reg dirty_w [0:3][0:1];
    reg dirty_r [0:3][0:1];
    reg LRU_w [0:3];
    reg LRU_r [0:3];
    // why can't I declare them in the same line ?

    wire [25:0] addr_tag;
    wire [1:0] addr_index;
    wire [1:0] addr_offset;
    wire sel_associative;
    assign addr_tag = proc_addr[29:4];
    assign addr_index = proc_addr[3:2];
    assign addr_offset = proc_addr[1:0];

    reg slot_num [0:3];

    integer i,j;
    
//==== combinational circuit ==============================
    
    always @(*) begin
        // default values
        for (i = 0; i<4; i=i+1) begin
            for (j = 0; j<2; j=j+1) begin
                valid_w [i][j] = valid_r [i][j];
                dirty_w [i][j] = dirty_r [i][j];
                data_w [i][j] = data_r [i][j];
                tag_w [i][j] = tag_r [i][j];
            end 
            LRU_w [i] = LRU_r [i];
            slot_num[i] = (valid_r[i][0]) ? 
                            (valid_r[i][1] ) ? LRU_r[i] : 1 
                            : 0;
        end
        proc_stall = 0;
        proc_rdata = 0;
        mem_read = 0;
        mem_write = 0;
        mem_addr = 0;
        mem_wdata = 0;
        // modifiable to save area ?

        state_w = state_r;

        case (state_r)
            S_IDLE:begin
                if(proc_read || proc_write) begin
                    // identify index, check valid first
                    // then check tag
                    if (valid_r[addr_index][0] && (tag_r[addr_index][0] == addr_tag) ) begin // valid & hit slot 0
                        LRU_w[addr_index] = 1; // set opposite
                            case (addr_offset) // *32
                                0: proc_rdata = data_r[addr_index][0][31 : 0];
                                1: proc_rdata = data_r[addr_index][0][63 : 32];
                                2: proc_rdata = data_r[addr_index][0][95 : 64];
                                3: proc_rdata = data_r[addr_index][0][127 : 96];
                            endcase
                    end

                    else if (valid_r[addr_index][1] && (tag_r[addr_index][1] == addr_tag) ) begin // valid & hit slot 1
                        LRU_w[addr_index] = 0; // set opposite
                            case (addr_offset) // *32
                                0: proc_rdata = data_r[addr_index][1][31 : 0];
                                1: proc_rdata = data_r[addr_index][1][63 : 32];
                                2: proc_rdata = data_r[addr_index][1][95 : 64];
                                3: proc_rdata = data_r[addr_index][1][127 : 96];
                            endcase
                    end
                    
                    // structure TBM
                    // no buffer for now
                    // else if ((!valid_r[addr_index][ LRU_r[addr_index] ]) || (!dirty_r[addr_index][ LRU_r[addr_index] ])) begin 
                    else  begin 
                    // clean or not valid, allocate directly
                        state_w = S_ALLOCATION;
                        proc_stall = 1;
                        mem_read = 1;
                        mem_addr = {addr_tag, addr_index};
                    end

                    //fetch with write allocation(fetch when write miss)
                    
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
                    
                    //proc_stall = 0;
                    // LRU_w[addr_index] = ~LRU_r[addr_index];
                    LRU_w[addr_index] = ~slot_num[addr_index];
                        case (addr_offset) // *32
                            0: proc_rdata = mem_rdata[31 : 0];
                            1: proc_rdata = mem_rdata[63 : 32];
                            2: proc_rdata = mem_rdata[95 : 64];
                            3: proc_rdata = mem_rdata[127 : 96];
                        endcase

                end
            end

        endcase
    end
    
//==== sequential circuit =================================
    always@( posedge clk ) begin
        if( proc_reset ) begin
            for (i = 0; i<4; i=i+1) begin
                for (j = 0; j<2; j=j+1) begin
                    valid_r[i][j] <= 0;
                    dirty_r[i][j] <= 0;
                end
                LRU_r[i] <= 0;
            end
            
            state_r <= S_IDLE;
        end

        else begin
            for (i = 0; i<4; i=i+1) begin
                for (j = 0; j<2; j=j+1) begin
                    valid_r [i][j] <= valid_w [i][j];
                    dirty_r [i][j] <= dirty_w [i][j];
                    data_r [i][j] <= data_w [i][j];
                    tag_r [i][j] <= tag_w [i][j];
                end      
                LRU_r[i] <= LRU_w[i];
            end

            state_r <= state_w;
        end
    end

endmodule
