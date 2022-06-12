module RISCV_Pipeline (
		// control interface
		clk            ,
		rst_n          ,
//---------I cache interface-------		
		ICACHE_ren     ,
		ICACHE_wen     ,
		ICACHE_addr    ,
		ICACHE_wdata   ,
		ICACHE_stall   ,
		ICACHE_rdata   ,
//---------D cache interface-------
		DCACHE_ren     ,
		DCACHE_wen     ,
		DCACHE_addr    ,
		DCACHE_wdata   ,
		DCACHE_stall   ,
		DCACHE_rdata   ,
        PC
);

input         clk, rst_n ;
// for mem_D
output        DCACHE_wen  ;  // mem_wen_D is high, CHIP writes data to D-mem
output        DCACHE_ren  ;
input         DCACHE_stall;
output [29:0] DCACHE_addr ;  // the specific address to fetch/store data 
output [31:0] DCACHE_wdata;  // data writing to D-mem 
input  [31:0] DCACHE_rdata;  // data reading from D-mem
// for mem_I
output        ICACHE_wen  ;
output        ICACHE_ren  ;
input         ICACHE_stall;
output [29:0] ICACHE_addr ;
output [31:0] ICACHE_wdata;
input  [31:0] ICACHE_rdata;
output [31:0] PC;

assign ICACHE_ren = 1;
assign ICACHE_wen = 0;

//---------module instantiation and module wire definition-------
wire [31:0] pc_IF_ID,inst_IF_ID;
wire [31:0] pc_jump;
wire haz_ID;
//assign haz_ID = 0;
wire EX_haz;
wire branch_stall;
wire IDIF_bubble;
wire [31:0] pc_jump_jal;
wire jump_jal;
wire jump ;
//wire ID_EX_bubble;
IF instfetch(
    .clk(clk),
    .rst_n(rst_n),
    .pc_jump(pc_jump),
    .ICACHE_addr(ICACHE_addr),
    .pc(pc_IF_ID),
    .inst(inst_IF_ID),
    .jump(jump),
    .ICACHE_rdata(ICACHE_rdata),
    .i_stall(ICACHE_stall),
    .d_stall(DCACHE_stall),
    .i_ID_haz(haz_ID),
    .i_EX_haz(EX_haz),
    .pc_top(PC),
    .i_bubble(IDIF_bubble),
    .i_pc_jump_jal(pc_jump_jal),
    .i_jump_jal(jump_jal)
);

wire [31:0] pc_ID_EX;
wire [31:0] imm_ID_EX;
wire [31:0] rs1_ID_EX,rs2_ID_EX;
wire [4:0] rd_addr_WB_ID;
wire [31:0] rd_data_WB_ID;
wire [4:0] rd_ID_EX;
wire [1:0] ctrl_wb_EX_MEM,ctrl_wb_ID_EX,ctrl_wb_MEM_WB;
wire [1:0] ctrl_mem_EX_MEM,ctrl_mem_ID_EX;
wire [5:0] ctrl_ex_ID_EX;
wire [4:0] rs1_addr_ID,rs2_addr_ID;
wire [31:0] forward_data_ID,forward_data_ID2;
wire forward_ID,forward_ID2;
wire ID_jalr,ID_branch;
wire [4:0] ID_EX_rs1_addr,ID_EX_rs2_addr;
wire [31:0] inst_ID_EX,inst_EX_MEM,inst_MEM_WB;

wire ID_EX_branch; 
BranchPredict branchpre(
    .i_ID_EX_branch(ID_EX_branch),
    .i_EX_jump(jump),
    .o_branch_stall(branch_stall)
);

wire ID_EX_jalr;

ID instdec(
    .clk(clk),
    .rst_n(rst_n),
    .i_inst(inst_IF_ID),
    .i_pc(pc_IF_ID),
    .o_pc(pc_ID_EX),
    .o_imm(imm_ID_EX),
    .o_rs1_data(rs1_ID_EX),
    .o_rs2_data(rs2_ID_EX),
    .i_rd_addr(rd_addr_WB_ID),
    .i_rd_data(rd_data_WB_ID),
    .o_rd_addr(rd_ID_EX),
    .o_ctrl_wb(ctrl_wb_ID_EX),
    .o_ctrl_mem(ctrl_mem_ID_EX),
    .o_ctrl_ex(ctrl_ex_ID_EX),
    .i_regwrite(regwrite_WB_ID),
    //.o_jump(jump),
    //.o_pc_jump(pc_jump),
    .stall(DCACHE_stall),
    .o_rs1_addr(rs1_addr_ID),
    .o_rs2_addr(rs2_addr_ID),
    .i_ID_haz(haz_ID),
    .i_forward(forward_ID),
    .i_forward_data(forward_data_ID),
    .i_forward2(forward_ID2),
    .i_forward_data2(forward_data_ID2),
    .o_ID_jalr(ID_jalr),
    .o_ID_EX_rs1_addr(ID_EX_rs1_addr),
    .o_ID_EX_rs2_addr(ID_EX_rs2_addr),
    .i_EX_haz(EX_haz),
    .o_inst(inst_ID_EX),//debug
    .o_ID_branch(ID_branch),
    .i_bubble(IDIF_bubble),
    .o_ID_EX_beq(ID_EX_beq),
    .o_ID_EX_bne(ID_EX_bne),
    .o_ID_EX_jalr(ID_EX_jalr),
    .o_jump_jal(jump_jal),
    .o_pc_jump_jal(pc_jump_jal)
    //.o_ID_EX_bubble(ID_EX_bubble)
);

wire [4:0] rd_EX_MEM;
wire [31:0] alu_EX_MEM;
wire [31:0] rs2_EX_MEM;
wire EX_stall;
wire EX_forward1,EX_forward2;
wire [31:0] EX_forward_data1,EX_forward_data2;
wire [31:0] alu_out_forward;

EX exe(
    .clk(clk),
    .rst_n(rst_n),
    //from ID
    .i_pc(pc_ID_EX),
    .i_imm(imm_ID_EX),
    .i_rs1_data(rs1_ID_EX),
    .i_rs2_data(rs2_ID_EX),
    .i_rd_addr(rd_ID_EX),
    .i_ctrl_wb(ctrl_wb_ID_EX),
    .i_ctrl_mem(ctrl_mem_ID_EX),
    .i_ctrl_ex(ctrl_ex_ID_EX),
    //to MEM
    .o_ctrl_mem(ctrl_mem_EX_MEM),
    .o_ctrl_wb(ctrl_wb_EX_MEM),
    .o_rd_addr(rd_EX_MEM),
    .o_alu_out(alu_EX_MEM),
    .o_rs2_data(rs2_EX_MEM),
    .stall(DCACHE_stall),
    //from EXforward
    .i_forward_data1(EX_forward_data1),
    .i_forward_data2(EX_forward_data2),
    .i_forward1(EX_forward1),
    .i_forward2(EX_forward2),
    .i_EX_haz(EX_haz),
    .i_inst(inst_ID_EX),
    .o_inst(inst_EX_MEM),
    .i_ID_EX_beq(ID_EX_beq),
    .i_ID_EX_bne(ID_EX_bne),
    .i_ID_EX_jalr(ID_EX_jalr),
    .o_pc_jump(pc_jump),
    .o_jump_jalr(jump_jalr),
    .o_jump_branch(jump_branch),
    .i_alu_out_forward(alu_out_forward)
    //.i_ID_EX_bubble(ID_EX_bubble)
);
assign jump = (jump_jalr || jump_branch);

assign IDIF_bubble = (branch_stall || ID_EX_jalr);
assign ID_EX_branch = (ID_EX_beq || ID_EX_bne);
wire [4:0] rd_MEM_WB;
wire [31:0] alu_MEM_WB;
wire [31:0] mem_MEM_WB;
wire MEM_stall;
MEM mem0(
    .clk(clk),
    .rst_n(rst_n),
    //from EX
    .i_ctrl_mem(ctrl_mem_EX_MEM),
    .i_ctrl_wb(ctrl_wb_EX_MEM),
    .i_rd_addr(rd_EX_MEM),
    .i_alu_out(alu_EX_MEM),
    .i_rs2_data(rs2_EX_MEM),
    //to WB
    .o_ctrl_wb(ctrl_wb_MEM_WB),
    .o_rd_addr(rd_MEM_WB),
    .o_alu_out(alu_MEM_WB),
    .o_DCACHE_rdata(mem_MEM_WB),
    //D cache
    .o_DCACHE_wen(DCACHE_wen),
    .o_DCACHE_addr(DCACHE_addr),
    .o_DCACHE_wdata(DCACHE_wdata),
    .i_DCACHE_rdata(DCACHE_rdata),
    .o_DCACHE_ren(DCACHE_ren),
    .stall(DCACHE_stall),
    .i_inst(inst_EX_MEM),
    .o_inst(inst_MEM_WB)
);

WB writeback(
    .clk(clk),
    .rst_n(rst_n),
    //from MEM
    .i_ctrl_wb(ctrl_wb_MEM_WB),
    .i_rd_addr(rd_MEM_WB),
    .i_alu_out(alu_MEM_WB),
    .i_DCACHE_rdata(mem_MEM_WB),
    //to ID
    .o_regwrite(regwrite_WB_ID),
    .o_rd_addr(rd_addr_WB_ID),
    .o_rd_data(rd_data_WB_ID),
    .i_inst(inst_MEM_WB)
);

IDForward idforwd(
    .i_ID_EX_rd_addr(rd_ID_EX),
    .i_ID_EX_ctrl_wb(ctrl_wb_ID_EX),//{memtoreg,regwrite}
    .i_EX_MEM_aluout(alu_EX_MEM),
    .i_EX_MEM_rd_addr(rd_EX_MEM),
    .i_MEM_WB_rd_addr(rd_MEM_WB),
    .i_EX_MEM_ctrl_wb(ctrl_wb_EX_MEM),
    .i_MEM_WB_ctrl_wb(ctrl_wb_MEM_WB),
    .i_WB_rd_data(rd_data_WB_ID),
    .i_ID_rs1_addr(rs1_addr_ID),
    .i_ID_rs2_addr(rs2_addr_ID),
    .o_ID_forward_data(forward_data_ID), //use this data as rs1 if o_ID_forward = 1
    .o_ID_forward(forward_ID),
    .o_ID_forward2(forward_ID2),
    .o_ID_forward_data2(forward_data_ID2),
    .i_ID_jalr(ID_jalr),
    .i_ID_branch(ID_branch),
    .o_ID_haz(haz_ID)  // need to repeat in IF.ID, and add bubble to EX 
);

EXForward exforwd(
    .i_ID_EX_rs1_addr(ID_EX_rs1_addr),
    .i_ID_EX_rs2_addr(ID_EX_rs2_addr),
    .i_EX_MEM_rd_addr(rd_EX_MEM),
    .i_MEM_WB_rd_addr(rd_MEM_WB),
    .i_EX_MEM_ctrl_wb(ctrl_wb_EX_MEM),//{memtoreg,regwrite}
    .i_MEM_WB_ctrl_wb(ctrl_wb_MEM_WB),
    .i_EX_MEM_aluout(alu_EX_MEM),
    .i_WB_rd_data(rd_data_WB_ID),
    .o_EX_forward1(EX_forward1),
    .o_EX_forward2(EX_forward2),
    .o_EX_forward_data1(EX_forward_data1),
    .o_EX_forward_data2(EX_forward_data2),
    .o_EX_haz(EX_haz),
    .i_ID_EX_ctrl_wb(ctrl_wb_ID_EX),
    .i_ID_jalr(ID_jalr),
    .i_ID_branch(ID_branch),
    .i_ID_EX_rd_addr(rd_ID_EX),
    .i_IF_ID_rs1_addr(rs1_addr_ID),
    .i_IF_ID_rs2_addr(rs2_addr_ID),
    .i_dstall(DCACHE_stall),
    .i_ID_EX_rs1_data(rs1_ID_EX),
    .i_ID_EX_rs2_data(rs2_ID_EX),
    .i_ctrl(ctrl_ex_ID_EX),
    .i_imm(imm_ID_EX),
    .o_alu_out(alu_out_forward)
);
    
endmodule



module IF (
    clk,rst_n,pc_jump,ICACHE_addr,pc,inst,jump,ICACHE_rdata,i_stall,d_stall,
    i_ID_haz,i_EX_haz,pc_top,i_bubble,i_jump_jal,i_pc_jump_jal
);

input i_ID_haz;
input clk,rst_n;
input jump; //jump = 1 --> pc_w = pc_jump
input [31:0] pc_jump; // jump location
output [29:0] ICACHE_addr;
output [31:0] pc; // pc : pc pass to next stage
input [31:0] ICACHE_rdata; //inst read from ICACHE
output [31:0] inst; // inst pass to next stage
input i_stall,d_stall;
input i_EX_haz;
output [31:0] pc_top;
input i_bubble;
input i_jump_jal;
input [31:0] i_pc_jump_jal;

//---------reg and wire definition-------
reg [31:0] inst_w,inst_r;
reg [31:0] pc_w,pc_r,pc_nxtstage;

//reg bubble;
//reg bubble_before;
reg jalr_before,jalr_2cycle;
reg jal_before;
//reg haz_before;

wire [6:0] opcode = inst_w[6:0];
wire [31:0] j_imm = {{12{inst_w[31]}},inst_w[19:12],inst_w[20],inst_w[30:25],inst_w[24:21],1'b0};
wire jal = (opcode ==  7'b1101111);
wire jalr = (opcode == 7'b1100111);
wire branch = (opcode == 7'b1100011);
//---------output assignment-------
assign inst = inst_r;
assign pc = pc_nxtstage;
assign ICACHE_addr = pc_r[31:2];
assign pc_top = pc_r;

//---------combinational circuit-------
wire [31:0] pc_notjump,pc_plus;
assign pc_notjump = pc_r + 4;
//pc_w
always @(*) begin
    if (jump) 
        pc_w = pc_jump;
    else if (i_jump_jal)
        pc_w = i_pc_jump_jal;
    else if(jalr_before || jalr || jal)begin
        pc_w = pc_r;
    end
    else begin
        pc_w = pc_notjump;
    end
end
//inst_w
always @(*) begin
    //if (bubble || i_stall || (bubble_before && haz_before)) 
    if (i_stall || i_bubble || jal_before) 
        inst_w = 32'h13; //NOP
    else
        inst_w = {ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};
end
localparam NOP = 32'h13;

//---------sequential circuit-------

reg jalr_before_w,jalr_2cycle_w,jal_before_w;
always @(*) begin
    if (i_ID_haz || i_EX_haz) begin
        jalr_before_w = jalr_before;
        jalr_2cycle_w = jalr_2cycle;
        jal_before_w = jal_before;
    end
    else begin
        jalr_before_w = jalr;
        jalr_2cycle_w = jalr_before;
        jal_before_w = jal;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)begin
        jalr_before <= 0;
        jalr_2cycle <=0;
        jal_before <= 0;
    end
    else begin
        jalr_before <= jalr_before_w;
        jalr_2cycle <= jalr_2cycle_w;
        jal_before <= jal_before_w;
    end
end

reg [31:0] pc_w_verified;
reg [31:0] inst_w_verified;
reg [31:0] pc_nxtstage_w;

always @(*) begin
    if (i_ID_haz || i_EX_haz)begin
        inst_w_verified = inst_r;
        pc_w_verified = pc_r;
        pc_nxtstage_w = pc_nxtstage;
    end

    else if (i_stall || jalr)begin
        if (jump)
            pc_w_verified = pc_w;
        else 
            pc_w_verified = pc_r;
        inst_w_verified = inst_w;
        pc_nxtstage_w = pc_r;
    end
    else if (d_stall)begin
        pc_w_verified = pc_r;
        inst_w_verified = inst_r;
        pc_nxtstage_w = pc_nxtstage; 
    end
    else begin
        pc_w_verified = pc_w;
        pc_nxtstage_w = pc_r;
        inst_w_verified = inst_w;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        pc_r <= 0;
        inst_r <= 0;
        pc_nxtstage <= 0;
    end
    else begin
        pc_r <= pc_w_verified;
        inst_r <= inst_w_verified;
        pc_nxtstage <= pc_nxtstage_w;
    end
end

// always @(posedge clk or negedge rst_n) begin
//     //bubble_before <= bubble;
//     //haz_before <= (i_ID_haz||i_EX_haz);
//     if (!rst_n) begin
//         pc_r <= 0;
//     end
//     else begin
//         if (i_ID_haz || i_EX_haz)begin
//             inst_r <= inst_r;
//             pc_r <= pc_r;
//             pc_nxtstage <= pc_nxtstage;
//         end

//         else if (i_stall || jalr)begin
//             if (jump)
//                 pc_r <= pc_w;
//             else 
//                 pc_r <= pc_r;
//             inst_r <= inst_w;
//             pc_nxtstage <= pc_r;
//         end
//         else if (d_stall)begin
//             pc_r <= pc_r;
//             inst_r <= inst_r;
//             pc_nxtstage <= pc_nxtstage; 
//         end
//         else begin
//             pc_r <= pc_w;
//             pc_nxtstage <= pc_r;
//             inst_r <= inst_w;
//         end
//     end
// end

endmodule

module ID (
    clk,
    rst_n,
    //from ID
    i_inst,
    i_pc,
    //to EX
    o_pc,
    o_imm,
    o_rs1_data,
    o_rs2_data,
    //from WB
    i_rd_addr,
    i_rd_data,
    //to EX
    o_rd_addr,
    o_ctrl_wb,
    o_ctrl_mem,
    o_ctrl_ex,
    //from WB ctrl
    i_regwrite,
    //to IF stage
    o_jump,
    o_pc_jump,
    //stall
    stall,
    // to jalrforward
    o_rs1_addr,
    o_rs2_addr,
    o_ID_jalr,
    o_ID_branch,
    //from jalrforward
    i_ID_haz,
    i_forward,
    i_forward_data,
    i_forward2,
    i_forward_data2,
    //to EXforward
    o_ID_EX_rs1_addr,
    o_ID_EX_rs2_addr,
    //from EXforward
    i_EX_haz,
    o_inst,
    o_ID_EX_beq,
    o_ID_EX_bne,
    o_ID_EX_jalr,
    i_bubble,
    o_jump_jal,
    o_pc_jump_jal
    //o_ID_EX_bubble
);
input clk;
input rst_n;
input [31:0] i_inst;
input [31:0] i_pc;
output reg [31:0] o_pc;
output reg [31:0] o_imm;
output reg [31:0] o_rs1_data,o_rs2_data;
input [4:0] i_rd_addr;
input [31:0] i_rd_data;
output reg [4:0] o_rd_addr;
output reg [1:0] o_ctrl_wb; //{memtoreg,regwrite} for next state
output reg [1:0] o_ctrl_mem; //{memwrite} for next state
output reg [5:0] o_ctrl_ex; //{alupc,aluctrl[3:0],alusrc} for next state
input i_regwrite; // i_regwrite = 1 when need to write back
output reg o_jump; //=1 when pc_nxt = pc_jump
output [31:0] o_pc_jump;
input stall;
output [4:0] o_rs1_addr,o_rs2_addr;
input i_forward,i_forward2;
input [31:0] i_forward_data,i_forward_data2;
input i_ID_haz;
output  o_ID_jalr;
output reg [4:0] o_ID_EX_rs1_addr,o_ID_EX_rs2_addr;
input i_EX_haz;
output reg [31:0] o_inst;
output  o_ID_branch;
output reg o_ID_EX_beq,o_ID_EX_bne;
output reg o_ID_EX_jalr;
input i_bubble;
output reg o_jump_jal;
output reg [31:0] o_pc_jump_jal;
//output reg o_ID_EX_bubble;
//---------reg and wire definition-------


reg jal;      // this stage //not used now
reg jalr;     // this stage
reg branch;   // this stage
reg memtoreg; // wb stage
reg memwrite; // mem stage
reg memread;  // mem stage
reg alusrc;   // ex stage
reg alupc;    // ex stage, alupc = 1 when jal or jalr(write pc+4 to reg)
reg regwrite; // wb stage
reg rs_equal; // this stage
reg [3:0] aluctrl; //ex stage

wire [31:0] i_imm,s_imm,b_imm,j_imm;
reg [31:0] imm_w;
wire [6:0] opcode ;
wire [2:0] funct3;
wire [4:0] rd_addr;
wire [4:0] rs1,rs2; 
wire bubble = (i_ID_haz || i_EX_haz || i_bubble);
wire [1:0] ctrl_wb_w = (i_ID_haz || i_EX_haz || i_bubble)? 0 : {memtoreg,regwrite};
wire [1:0] ctrl_mem_w = (i_ID_haz || i_EX_haz || i_bubble)? 0 : {memread,memwrite};
wire [5:0] ctrl_ex_w = (i_ID_haz || i_EX_haz || i_bubble)? 0 : {alupc,aluctrl,alusrc};
wire [31:0] rs1_data,rs2_data;

reg [31:0] rs1_data_revised,rs2_data_revised;

reg_file reg0(
                .clk  (clk) , 
                .rst_n(rst_n) , 
                .wen  (i_regwrite) , 
                .a1   (rs1) , 
                .a2   (rs2) , 
                .aw   (i_rd_addr) , 
                .d    (i_rd_data) , 
                .q1   (rs1_data) , 
                .q2   (rs2_data) );

assign i_imm = {{21{i_inst[31]}},i_inst[30:25],i_inst[24:21],i_inst[20]};
assign s_imm = {{21{i_inst[31]}},i_inst[30:25],i_inst[11:8],i_inst[7]};
assign b_imm = {{20{i_inst[31]}},i_inst[7],i_inst[30:25],i_inst[11:8],1'b0};
assign j_imm = {{12{i_inst[31]}},i_inst[19:12],i_inst[20],i_inst[30:25],i_inst[24:21],1'b0};

assign opcode = i_inst[6:0];
assign funct3 = i_inst[14:12];
assign rd_addr = i_inst[11:7];
assign rs1 = i_inst[19:15];
assign rs2 = i_inst[24:20];
assign o_rs1_addr = rs1;
assign o_rs2_addr = rs2;
assign o_ID_jalr = jalr;
assign o_ID_branch = branch;
//aluctrl param
localparam ALU_ADD = 4'b0000;
localparam ALU_SUB = 4'b0001;
localparam ALU_AND = 4'b0010;
localparam ALU_OR =  4'b0011;
localparam ALU_SLT = 4'b0100;
localparam ALU_XOR = 4'b0101;
localparam ALU_SLL = 4'b0110;
localparam ALU_SRA = 4'b0111;
localparam ALU_SRL = 4'b1000;

always @(*) begin
    if(i_forward)
        rs1_data_revised = i_forward_data;
    else if (i_rd_addr && i_rd_addr == rs1 && i_regwrite)
        rs1_data_revised = i_rd_data;
    else 
        rs1_data_revised = rs1_data;
end
always @(*) begin
    if(i_forward2)
        rs2_data_revised = i_forward_data2;
    else if (i_rd_addr && i_rd_addr == rs2 && i_regwrite)
        rs2_data_revised = i_rd_data;
    else
        rs2_data_revised = rs2_data;
end

//ctrl and imm
always @(*) begin
    jal = 0;
    jalr = 0;
    branch = 0;
    memtoreg = 0;
    memwrite = 0;
    alusrc = 0;
    regwrite = 0;
    memread = 0;
    alupc = 0;
    case (opcode)
        7'b0110011:begin//arith
            imm_w = i_imm;//dont care
            regwrite = 1;
        end 
        7'b0010011:begin //arith,i type
            imm_w = i_imm;
            alusrc = 1;
            regwrite = 1;
        end
        7'b0000011:begin//load
            imm_w = i_imm;
            alusrc = 1;
            memtoreg = 1;
            regwrite = 1;
            memread = 1;
        end
        7'b0100011:begin//store
            imm_w = s_imm;
            alusrc = 1;
            memwrite = 1;
        end
        7'b1100011:begin//branch
            imm_w = b_imm;
            branch = 1;
            alusrc = 1;
        end
        7'b1100111:begin//jalr
            imm_w = i_imm;
            jalr = 1;
            regwrite = 1;
            alupc = 1;
        end
        7'b1101111:begin//jal
            imm_w = j_imm;
            jal = 1;
            regwrite = 1;
            alupc = 1;
        end
        default: begin
            imm_w = 0;
        end
    endcase
end
always @(*) begin
    o_jump_jal = jal;
    o_pc_jump_jal = i_pc + imm_w;
end

//aluctrl
always @(*) begin
    case (opcode)
        7'b0000011:aluctrl = ALU_ADD; // load
        7'b0100011:aluctrl = ALU_ADD; // store
        7'b1100111:aluctrl = ALU_ADD; //jalr
        7'b1101111:aluctrl = ALU_ADD; //jal,dont care
        //7'b1100011:aluctrl = ALU_SUB; // branch,actually dont care
        default: begin //arith
            case (funct3)
                3'b000: aluctrl = (i_inst[30] && opcode[5])? ALU_SUB:ALU_ADD;
                3'b001: aluctrl = ALU_SLL;
                3'b010: aluctrl = ALU_SLT;
                3'b100: aluctrl = ALU_XOR;
                3'b101: aluctrl = (i_inst[30])? ALU_SRA : ALU_SRL;
                3'b110: aluctrl = ALU_OR;
                3'b111: aluctrl = ALU_AND;
                default: aluctrl = 0;
            endcase
        end
    endcase
end

reg [31:0] sum_pc_a;
//o_pc_jump
always @(*) begin
    if(jalr)begin
        sum_pc_a = rs1_data_revised;
    end
    else 
        sum_pc_a = i_pc;
end
assign o_pc_jump = sum_pc_a + imm_w;

//o_jump
always @(*) begin
    rs_equal = (rs1_data_revised == rs2_data_revised);
    o_jump = (jalr || (branch && (funct3[0] ^ rs_equal))  && !i_ID_haz); //funct3[0] : BEQ = 0, BNE = 1
end

reg [31:0] o_pc_w,o_imm_w,o_rs1_data_w,o_rs2_data_w;
reg [4:0] o_rd_addr_w;
reg [5:0] o_ctrl_ex_w;
reg [1:0] o_ctrl_mem_w;
reg [1:0] o_ctrl_wb_w;
reg [31:0] o_inst_w;
reg [4:0] o_ID_EX_rs1_addr_w,o_ID_EX_rs2_addr_w;
reg o_ID_EX_beq_w,o_ID_EX_bne_w;
reg o_ID_EX_jalr_w;
//reg o_ID_EX_bubble_w;

always @(*) begin
    if (stall)begin
        o_ID_EX_beq_w = o_ID_EX_beq;
        o_ID_EX_bne_w = o_ID_EX_bne;
        o_pc_w = o_pc;
        o_imm_w = o_imm;
        o_rs1_data_w = o_rs1_data;
        o_rs2_data_w = o_rs2_data;
        o_rd_addr_w = o_rd_addr;
        o_ctrl_wb_w = o_ctrl_wb;
        o_ctrl_ex_w = o_ctrl_ex;
        o_ctrl_mem_w = o_ctrl_mem;
        o_inst_w = o_inst;
        o_ID_EX_rs1_addr_w = o_ID_EX_rs1_addr;
        o_ID_EX_rs2_addr_w = o_ID_EX_rs2_addr;
        o_ID_EX_jalr_w = o_ID_EX_jalr;
        //o_ID_EX_bubble_w = o_ID_EX_bubble;
    end
    else begin
        o_pc_w = i_pc;
        o_imm_w = imm_w;
        o_rs1_data_w = rs1_data_revised;
        o_rs2_data_w = rs2_data_revised;
        o_rd_addr_w = rd_addr;
        o_ctrl_wb_w = ctrl_wb_w;
        o_ctrl_ex_w = ctrl_ex_w;
        o_ctrl_mem_w = ctrl_mem_w;
        //o_ID_EX_bubble_w = bubble;
        if (i_ID_haz || i_EX_haz || i_bubble) begin
            o_inst_w = 32'h13;
            o_ID_EX_jalr_w = 0;
            o_ID_EX_beq_w = 0;
            o_ID_EX_bne_w = 0;
        end
        else begin
            o_inst_w = i_inst;
            o_ID_EX_beq_w = (branch && !funct3[0]);
            o_ID_EX_bne_w = (branch &&  funct3[0]);
            o_ID_EX_jalr_w = jalr;
        end
        o_ID_EX_rs1_addr_w = rs1;
        o_ID_EX_rs2_addr_w = rs2;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)begin
        o_pc <= 0;
        o_imm <= 0;
        o_rs1_data <= 0;
        o_rs2_data <= 0;
        o_rd_addr <= 0;
        o_ctrl_wb <= 0;
        o_ctrl_ex <= 0;
        o_ctrl_mem <= 0;
        o_inst <= 0;
        o_ID_EX_rs1_addr <= 0;
        o_ID_EX_rs2_addr <= 0;
        o_ID_EX_beq <= 0;
        o_ID_EX_bne <= 0;
        o_ID_EX_jalr <= 0;
        //o_ID_EX_bubble <= 0;
    end
    else begin
        o_pc <= o_pc_w;
        o_imm <= o_imm_w;
        o_rs1_data <= o_rs1_data_w;
        o_rs2_data <= o_rs2_data_w;
        o_rd_addr <= o_rd_addr_w;
        o_ctrl_wb <= o_ctrl_wb_w;
        o_ctrl_ex <= o_ctrl_ex_w;
        o_ctrl_mem <= o_ctrl_mem_w;
        o_inst <= o_inst_w;
        o_ID_EX_rs1_addr <= o_ID_EX_rs1_addr_w;
        o_ID_EX_rs2_addr <= o_ID_EX_rs2_addr_w;
        o_ID_EX_beq <= o_ID_EX_beq_w;
        o_ID_EX_bne <= o_ID_EX_bne_w;
        o_ID_EX_jalr <= o_ID_EX_jalr_w;
        //o_ID_EX_bubble <= o_ID_EX_bubble_w;
    end
end
    
endmodule


module EX (
    clk,
    rst_n,
    //from ID
    i_pc,
    i_imm,
    i_rs1_data,
    i_rs2_data,
    i_rd_addr,
    i_ctrl_wb,
    i_ctrl_mem,
    i_ctrl_ex,
    //to MEM
    o_ctrl_mem,
    o_ctrl_wb,
    o_rd_addr,
    o_alu_out,
    o_rs2_data,
    //stall
    stall,
    //from EXforward
    i_forward1,
    i_forward2,
    i_forward_data1,
    i_forward_data2,
    i_EX_haz,
    i_inst,
    o_inst,
    i_ID_EX_beq,
    i_ID_EX_bne,
    i_ID_EX_jalr,
    o_pc_jump,
    o_jump_jalr,
    o_jump_branch,
    i_alu_out_forward
    //i_ID_EX_bubble
);

input clk,rst_n;
input [31:0] i_pc;
input [31:0] i_imm;
input [31:0] i_rs1_data,i_rs2_data;
input [4:0] i_rd_addr;
input [5:0] i_ctrl_ex;
input [1:0] i_ctrl_mem;
input [1:0] i_ctrl_wb;
input stall;
input i_forward1,i_forward2;
input [31:0] i_forward_data1,i_forward_data2;
input i_EX_haz;
input [31:0] i_inst;
input i_ID_EX_beq,i_ID_EX_bne;
input i_ID_EX_jalr;
input [31:0] i_alu_out_forward;
//input i_ID_EX_bubble;

output reg [1:0] o_ctrl_mem;
output reg [1:0] o_ctrl_wb;
output reg [4:0] o_rd_addr;
output reg [31:0] o_rs2_data;
output reg [31:0] o_alu_out;
output reg [31:0] o_inst;
output reg [31:0] o_pc_jump;
output reg o_jump_jalr,o_jump_branch;

reg [31:0] alu_in2,alu_in1;
reg [31:0] alu_out;
wire alusrc = i_ctrl_ex[0];
wire alupc = i_ctrl_ex[5];
wire [3:0] aluctrl = i_ctrl_ex[4:1];

//ALU alu0(.in1(alu_in1),.in2(alu_in2),.out(alu_out),.ctrl(aluctrl));

reg [31:0] rs2_data_revised,rs1_data_revised;
always @(*) begin
    if(i_forward2)
        rs2_data_revised = i_forward_data2;
    else 
        rs2_data_revised = i_rs2_data;
end
always @(*) begin
    if(i_forward1)
        rs1_data_revised = i_forward_data1;
    else
        rs1_data_revised = i_rs1_data;
end 

always @(*) begin
    begin
        if(alupc)begin
            alu_out = i_pc + 4;
        end
        else begin
            alu_out = i_alu_out_forward;
        end
    end
end


wire rs_equal = (i_rs1_data == i_rs2_data);
always @(*) begin
    if((i_ID_EX_beq && rs_equal) || (i_ID_EX_bne && !rs_equal))
        o_jump_branch = 1;
    else 
        o_jump_branch = 0;
end
always @(*) begin
    o_jump_jalr = i_ID_EX_jalr;
end
wire [31:0] pc_jump_a = (i_ID_EX_jalr)? i_rs1_data : i_pc;
always @(*) begin
    o_pc_jump = pc_jump_a + i_imm;
end

reg [1:0] o_ctrl_mem_w;
reg [1:0] o_ctrl_wb_w;
reg [4:0] o_rd_addr_w;
reg [31:0] o_alu_out_w;
reg [31:0] o_rs2_data_w;
reg [31:0] o_inst_w;

always @(*) begin
    if (stall)begin
        o_ctrl_mem_w = o_ctrl_mem;
        o_ctrl_wb_w = o_ctrl_wb;
        o_rd_addr_w = o_rd_addr;
        o_alu_out_w = o_alu_out;
        o_rs2_data_w = o_rs2_data;
        o_inst_w = o_inst;
    end
    else begin
        o_ctrl_mem_w = i_ctrl_mem;
        o_ctrl_wb_w = i_ctrl_wb;
        o_rd_addr_w = i_rd_addr;
        o_alu_out_w = alu_out;
        o_rs2_data_w = rs2_data_revised;
        o_inst_w = i_inst;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)begin
        o_ctrl_mem <= 0;
        o_ctrl_wb <= 0;
        o_rd_addr <= 0;
        o_alu_out <= 0;
        o_rs2_data <= 0;
        o_inst <= 0;
    end
    else begin
        o_ctrl_mem <= o_ctrl_mem_w;
        o_ctrl_wb <= o_ctrl_wb_w;
        o_rd_addr <= o_rd_addr_w;
        o_alu_out <= o_alu_out_w;
        o_rs2_data <= o_rs2_data_w;
        o_inst <= o_inst_w;
    end
    
end
    
endmodule

module MEM (
    clk,
    rst_n,
    //from EX
    i_ctrl_mem,
    i_ctrl_wb,
    i_rd_addr,
    i_alu_out,
    i_rs2_data,
    //to WB
    o_ctrl_wb,
    o_rd_addr,
    o_alu_out,
    o_DCACHE_rdata,
    //D cache
    o_DCACHE_wen,
    o_DCACHE_addr,
    o_DCACHE_wdata,
    i_DCACHE_rdata,
    o_DCACHE_ren,
    //stall
    stall,
    i_inst,
    o_inst
);
input clk,rst_n;
input [1:0] i_ctrl_mem;//{memread,memwrite}
input [1:0] i_ctrl_wb;
input [4:0] i_rd_addr;
input [31:0] i_alu_out;
input [31:0] i_rs2_data;
input stall;
input [31:0] i_inst;

output reg [1:0] o_ctrl_wb;
output reg [4:0] o_rd_addr;
output reg [31:0] o_alu_out;
output reg [31:0] o_inst;

output reg [31:0] o_DCACHE_rdata;
output o_DCACHE_wen,o_DCACHE_ren;
output [29:0] o_DCACHE_addr;
output [31:0] o_DCACHE_wdata;
input [31:0] i_DCACHE_rdata;

assign o_DCACHE_wen = i_ctrl_mem[0];
assign o_DCACHE_ren = i_ctrl_mem[1];
assign o_DCACHE_addr = i_alu_out[31:2];
assign o_DCACHE_wdata = {i_rs2_data[7:0],i_rs2_data[15:8],i_rs2_data[23:16],i_rs2_data[31:24]};

reg [1:0] o_ctrl_wb_w;
reg [4:0] o_rd_addr_w;
reg [31:0] o_alu_out_w;
reg [31:0] o_DCACHE_rdata_w;
reg [31:0] o_inst_w;

always @(*) begin
    if (stall)begin
        o_ctrl_wb_w = o_ctrl_wb;
        o_rd_addr_w = o_rd_addr;
        o_alu_out_w = o_alu_out;
        o_DCACHE_rdata_w = o_DCACHE_rdata;
        o_inst_w = o_inst;
    end
    else begin
        o_ctrl_wb_w = i_ctrl_wb;
        o_rd_addr_w = i_rd_addr;
        o_alu_out_w = i_alu_out;
        o_DCACHE_rdata_w = {i_DCACHE_rdata[7:0],i_DCACHE_rdata[15:8],i_DCACHE_rdata[23:16],i_DCACHE_rdata[31:24]};
        o_inst_w = i_inst;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)begin
        o_ctrl_wb <= 0;
        o_rd_addr <= 0;
        o_alu_out <= 0;
        o_DCACHE_rdata <= 0;
        o_inst <= 0;
    end
    else begin
        o_ctrl_wb <= o_ctrl_wb_w;
        o_rd_addr <= o_rd_addr_w;
        o_alu_out <= o_alu_out_w;
        o_DCACHE_rdata <= o_DCACHE_rdata_w;
        o_inst <= o_inst_w;
    end
end
    
endmodule

module WB (
    clk,
    rst_n,
    //from MEM
    i_ctrl_wb, //{memtoreg,regwrite}
    i_rd_addr,
    i_alu_out,
    i_DCACHE_rdata,
    //to ID
    o_regwrite,
    o_rd_addr,
    o_rd_data,
    i_inst
);

input clk,rst_n;
input [1:0] i_ctrl_wb;
input [4:0] i_rd_addr;
input [31:0] i_alu_out;
input [31:0] i_DCACHE_rdata;
input [31:0] i_inst;

output o_regwrite;
output [4:0] o_rd_addr;
output [31:0] o_rd_data;

wire memtoreg = i_ctrl_wb[1];

assign o_regwrite = i_ctrl_wb[0];
assign o_rd_addr = i_rd_addr;
assign o_rd_data = (memtoreg)? i_DCACHE_rdata:i_alu_out;
    
endmodule



module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                /*
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
                */
                mem[i] <= 32'h0;
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module ALU (
    in1,in2,out,ctrl
);
input [31:0] in1,in2;
output reg [31:0] out;
input [3:0] ctrl;

//ctrl param
localparam ALU_ADD = 4'b0000;
localparam ALU_SUB = 4'b0001;
localparam ALU_AND = 4'b0010;
localparam ALU_OR =  4'b0011;
localparam ALU_SLT = 4'b0100;
localparam ALU_XOR = 4'b0101;
localparam ALU_SLL = 4'b0110;
localparam ALU_SRA = 4'b0111;
localparam ALU_SRL = 4'b1000;
wire [31:0] sub = in1 - in2;

always @(*) begin
    case (ctrl)
        ALU_ADD: out = in1 + in2;
        ALU_SUB: out = sub;
        ALU_AND: out = in1 & in2;
        ALU_OR : out = in1 | in2;
        ALU_SLT: out = (sub[31]);
        ALU_XOR: out = in1 ^ in2;
        ALU_SLL: out = in1 << in2;
        ALU_SRA: out = $signed(in1) >>> in2;
        ALU_SRL: out = in1 >> in2; 
        default: out = in1 + in2; 
    endcase
end
    
endmodule

module ALU_simple (
    in1,in2,out,ctrl
);
input [31:0] in1,in2;
output reg [31:0] out;
input [3:0] ctrl;

//ctrl param
localparam ALU_ADD = 4'b0000;
localparam ALU_SUB = 4'b0001;
localparam ALU_SLT = 4'b0100;
reg [31:0] in2_modify;
wire [31:0] in2_neg = ~in2 + 1;
wire [31:0] sum = in1 + in2_modify;

always @(*) begin
    case (ctrl)
        ALU_ADD:begin
            in2_modify = in2;
            out = sum;
        end 
        ALU_SUB:begin
            in2_modify = in2_neg;
            out = sum;
        end 
        ALU_SLT:begin
            in2_modify = in2_neg;
            out = (sum[31]);
        end 
        default: begin
            in2_modify = in2;
            out = sum;
        end  
    endcase
end
    
endmodule

module ALU_fast (
    in1,in2,out,ctrl
);
input [31:0] in1,in2;
output reg [31:0] out;
input [3:0] ctrl;

//ctrl param
localparam ALU_ADD = 4'b0000;
localparam ALU_SUB = 4'b0001;
localparam ALU_AND = 4'b0010;
localparam ALU_OR =  4'b0011;
localparam ALU_SLT = 4'b0100;
localparam ALU_XOR = 4'b0101;
localparam ALU_SLL = 4'b0110;
localparam ALU_SRA = 4'b0111;
localparam ALU_SRL = 4'b1000;

always @(*) begin
    case (ctrl)
        ALU_AND: out = in1 & in2;
        ALU_OR : out = in1 | in2;
        ALU_XOR: out = in1 ^ in2;
        ALU_SLL: out = in1 << in2;
        ALU_SRA: out = $signed(in1) >>> in2;
        ALU_SRL: out = in1 >> in2; 
        default: out = in1 & in2; 
    endcase
end
    
endmodule

module IDForward (
    i_ID_EX_rd_addr,
    i_ID_EX_ctrl_wb,//{memtoreg,regwrite}
    i_EX_MEM_aluout,
    i_EX_MEM_rd_addr,
    i_MEM_WB_rd_addr,
    i_EX_MEM_ctrl_wb,
    i_MEM_WB_ctrl_wb,
    i_WB_rd_data,
    i_ID_rs1_addr,
    i_ID_rs2_addr,
    o_ID_forward_data, //use this data as rs1 if o_ID_forward = 1
    o_ID_forward,
    o_ID_forward_data2,
    o_ID_forward2,
    i_ID_jalr,
    i_ID_branch,
    o_ID_haz // do not happen in real life? (do not load an address then jalr to it normally) // need to repeat in IF.ID, and add bubble to EX 
);

input [31:0] i_WB_rd_data;
input [31:0] i_EX_MEM_aluout;
input [4:0] i_EX_MEM_rd_addr,i_MEM_WB_rd_addr,i_ID_EX_rd_addr;
input [1:0] i_ID_EX_ctrl_wb,i_EX_MEM_ctrl_wb,i_MEM_WB_ctrl_wb;
input [4:0] i_ID_rs1_addr,i_ID_rs2_addr;
output reg [31:0] o_ID_forward_data,o_ID_forward_data2;
output reg o_ID_forward,o_ID_forward2;
output o_ID_haz;
input i_ID_jalr;
input i_ID_branch;
reg haz1,haz2;
assign o_ID_haz = haz1 || haz2;
always @(*) begin
    o_ID_forward_data = 0;
    o_ID_forward = 0;
    haz1 = 0;
    if(i_ID_jalr || i_ID_branch)begin
        if(i_ID_rs1_addr) begin
            if(i_ID_EX_ctrl_wb[0] && (i_ID_EX_rd_addr == i_ID_rs1_addr))begin
                //$display("how come? jalr right after assign that reg");
                haz1 = 1;
            end
            else if (i_EX_MEM_ctrl_wb[0] && (i_EX_MEM_rd_addr == i_ID_rs1_addr))begin
                if(i_EX_MEM_ctrl_wb[1]) begin
                    //$display("how come? jalr to a load data");
                    haz1 = 1;
                end
                else begin
                    o_ID_forward = 1;
                    o_ID_forward_data = i_EX_MEM_aluout;
                end
            end
            else if (i_MEM_WB_ctrl_wb[0] && (i_MEM_WB_rd_addr == i_ID_rs1_addr))begin
                o_ID_forward = 1;
                o_ID_forward_data = i_WB_rd_data;
            end
        end
    end
end

always @(*) begin
    o_ID_forward_data2 = 0;
    o_ID_forward2 = 0;
    haz2 = 0;
    if(i_ID_branch)begin
        if(i_ID_rs2_addr) begin
            if(i_ID_EX_ctrl_wb[0] && (i_ID_EX_rd_addr == i_ID_rs2_addr))begin
                //$display("how come? jalr right after assign that reg");
                haz2 = 1;
            end
            else if (i_EX_MEM_ctrl_wb[0] && (i_EX_MEM_rd_addr == i_ID_rs2_addr))begin
                if(i_EX_MEM_ctrl_wb[1]) begin
                    //$display("how come? jalr to a load data");
                    haz2 = 1;
                end
                else begin
                    o_ID_forward2 = 1;
                    o_ID_forward_data2 = i_EX_MEM_aluout;
                end
            end
            else if (i_MEM_WB_ctrl_wb[0] && (i_MEM_WB_rd_addr == i_ID_rs2_addr))begin
                o_ID_forward2 = 1;
                o_ID_forward_data2 = i_WB_rd_data;
            end
        end
    end
end
    
endmodule

module EXForward (
    i_ID_EX_rs1_addr,
    i_ID_EX_rs2_addr,
    i_ID_EX_ctrl_wb,
    i_ID_jalr,
    i_ID_branch,
    i_ID_EX_rd_addr,
    i_IF_ID_rs1_addr,
    i_IF_ID_rs2_addr,
    i_EX_MEM_rd_addr,
    i_MEM_WB_rd_addr,
    i_EX_MEM_ctrl_wb,//{memtoreg,regwrite}
    i_MEM_WB_ctrl_wb,
    i_EX_MEM_aluout,
    i_WB_rd_data,
    o_EX_forward1,
    o_EX_forward2,
    o_EX_forward_data1,
    o_EX_forward_data2,
    o_EX_haz,
    i_dstall,
    i_ID_EX_rs1_data,
    i_ID_EX_rs2_data,
    i_ctrl,
    i_imm,
    o_alu_out
);

input [4:0] i_ID_EX_rs1_addr,i_ID_EX_rs2_addr;
input [4:0] i_IF_ID_rs1_addr,i_IF_ID_rs2_addr;
input [4:0] i_EX_MEM_rd_addr,i_MEM_WB_rd_addr;
input [1:0] i_EX_MEM_ctrl_wb,i_MEM_WB_ctrl_wb;
output reg o_EX_forward1,o_EX_forward2;
output reg [31:0] o_EX_forward_data1,o_EX_forward_data2;
output o_EX_haz;
input [31:0] i_EX_MEM_aluout;
input [31:0] i_WB_rd_data;
input i_ID_jalr,i_ID_branch;
input [4:0] i_ID_EX_rd_addr;
input [1:0] i_ID_EX_ctrl_wb;
input i_dstall;
input [31:0] i_ID_EX_rs1_data,i_ID_EX_rs2_data;
input [5:0] i_ctrl;
input [31:0] i_imm;
output reg [31:0] o_alu_out;

wire alusrc = i_ctrl[0];
wire alupc = i_ctrl[5];
wire [3:0] aluctrl = i_ctrl[4:1];

reg haz1,haz2;
wire [31:0] rs2_modify;
wire [31:0] alu_in2;

wire [31:0] rs1_exmem_out,rs1_memwb_out,exmem_rs2_out,memwb_rs2_out,memwb_exmem_out,exmem_memwb_out,rs1_rs2_out;
reg [31:0] rs1_data_revised,rs2_data_revised;
wire [31:0] rs1_rs2_out_revised;
reg rs1_mem,rs2_mem,rs1_wb,rs2_wb;

ALU_fast   rs1_rs2_revised    (.in1(rs1_data_revised),.in2(alu_in2),     .out(rs1_rs2_out_revised),    .ctrl(aluctrl));
ALU_simple rs1_rs2     (.in1(i_ID_EX_rs1_data),.in2(rs2_modify),     .out(rs1_rs2_out),    .ctrl(aluctrl));
ALU_simple rs1_exmem   (.in1(i_ID_EX_rs1_data),.in2(i_EX_MEM_aluout),.out(rs1_exmem_out),  .ctrl(aluctrl));
ALU_simple rs1_memwb   (.in1(i_ID_EX_rs1_data),.in2(i_WB_rd_data),   .out(rs1_memwb_out),  .ctrl(aluctrl));
ALU_simple exmem_rs2   (.in1(i_EX_MEM_aluout),.in2(rs2_modify),      .out(exmem_rs2_out),  .ctrl(aluctrl));
ALU_simple memwb_rs2   (.in1(i_WB_rd_data),   .in2(rs2_modify),      .out(memwb_rs2_out),  .ctrl(aluctrl));
ALU_simple memwb_exmem (.in1(i_WB_rd_data),   .in2(i_EX_MEM_aluout), .out(memwb_exmem_out),.ctrl(aluctrl));
ALU_simple exmem_memwb (.in1(i_EX_MEM_aluout),.in2(i_WB_rd_data),    .out(exmem_memwb_out),.ctrl(aluctrl));
assign o_EX_haz = haz1 || haz2;
assign alu_in2 = (alusrc)? i_imm : rs2_data_revised;
assign rs2_modify = (alusrc)? i_imm : i_ID_EX_rs2_data;

localparam ALU_ADD = 4'b0000;
localparam ALU_SUB = 4'b0001;
localparam ALU_SLT = 4'b0100;

always @(*) begin
    if ((aluctrl == ALU_ADD) || (aluctrl == ALU_SUB) || (aluctrl == ALU_SLT))begin
        if (alusrc)begin
            if(rs1_mem)begin
                o_alu_out = exmem_rs2_out;
            end
            else if (rs1_wb)begin
                o_alu_out = memwb_rs2_out;
            end
            else begin
                o_alu_out = rs1_rs2_out;
            end
        end
        else begin
            case ({rs1_mem,rs2_mem,rs1_wb,rs2_wb})
                4'b0000:begin
                    o_alu_out = rs1_rs2_out;
                end 
                4'b0100:begin
                    o_alu_out = rs1_exmem_out;
                end
                4'b1000:begin
                    o_alu_out = exmem_rs2_out;
                end
                4'b0010:begin
                    o_alu_out = memwb_rs2_out;
                end
                4'b0001:begin
                    o_alu_out = rs1_memwb_out;
                end
                4'b1001:begin
                    o_alu_out = exmem_memwb_out;
                end
                4'b0110:begin
                    o_alu_out = memwb_exmem_out;
                end
                default: o_alu_out = rs1_rs2_out;
            endcase
        end
    end
    else begin
        o_alu_out = rs1_rs2_out_revised;
    end
end


always @(*) begin
    o_EX_forward1 = 0;
    rs1_mem = 0;
    rs1_wb = 0;
    rs1_data_revised = i_ID_EX_rs1_data;
    o_EX_forward_data1 = 0; //default value, not used 
    if (i_ID_EX_rs1_addr) begin // not x0
        if((i_ID_EX_rs1_addr == i_EX_MEM_rd_addr) && i_EX_MEM_ctrl_wb[0])begin
            o_EX_forward1 = 1;
            o_EX_forward_data1 = i_EX_MEM_aluout;
            rs1_data_revised = i_EX_MEM_aluout;
            rs1_mem = 1;
        end
        else if ((i_ID_EX_rs1_addr == i_MEM_WB_rd_addr) && i_MEM_WB_ctrl_wb[0])begin
            o_EX_forward1 = 1;
            o_EX_forward_data1 = i_WB_rd_data;
            rs1_data_revised = i_WB_rd_data;
            rs1_wb = 1;
        end
    end
end

always @(*) begin
    o_EX_forward2 = 0;
    o_EX_forward_data2 = i_EX_MEM_aluout;
    rs2_data_revised = i_ID_EX_rs2_data;
    rs2_mem = 0;
    rs2_wb = 0;
    if (i_ID_EX_rs2_addr) begin
        if((i_ID_EX_rs2_addr == i_EX_MEM_rd_addr) && i_EX_MEM_ctrl_wb[0])begin
            o_EX_forward2 = 1;
            o_EX_forward_data2 = i_EX_MEM_aluout;
            rs2_data_revised = i_EX_MEM_aluout;
            rs2_mem = 1;
        end
        else if ((i_ID_EX_rs2_addr == i_MEM_WB_rd_addr) && i_MEM_WB_ctrl_wb[0])begin
            o_EX_forward2 = 1;
            o_EX_forward_data2 = i_WB_rd_data;
            rs2_data_revised = i_WB_rd_data;
            rs2_wb = 1;
        end
    end
end

always @(*) begin
    haz1 = 0;
    haz2 = 0;
    if(i_IF_ID_rs1_addr)begin
        if((i_IF_ID_rs1_addr == i_ID_EX_rd_addr) && i_ID_EX_ctrl_wb[1] && !i_ID_jalr && !i_ID_branch)begin
            haz1 = 1;
        end
        else if ((i_IF_ID_rs1_addr == i_EX_MEM_rd_addr) && i_dstall) begin
            haz1 = 1;
        end
    end
    if(i_IF_ID_rs2_addr)begin
        if((i_IF_ID_rs2_addr == i_ID_EX_rd_addr) && i_ID_EX_ctrl_wb[1] && !i_ID_jalr && !i_ID_branch)begin
            haz2 = 1;
        end
        else if ((i_IF_ID_rs2_addr == i_EX_MEM_rd_addr) && i_dstall) begin
            haz2 = 1;
        end
    end
end
    
endmodule

module BranchPredict (
    i_ID_EX_branch,
    i_EX_jump,
    o_branch_stall
);
input i_ID_EX_branch,i_EX_jump;
output o_branch_stall;

assign o_branch_stall = i_ID_EX_branch && i_EX_jump;
    
endmodule