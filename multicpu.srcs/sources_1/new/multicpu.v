`timescale 1ns / 1ps
module pc(
     input clk, rst, PCWre,
	input [1:0] PCSrc,
	input [31:0] immediate, readData1, addr,
	output reg[31:0] pcout
    );
	initial begin
		pcout = 0;
	end
	always@(negedge clk or negedge rst) begin
		if (rst == 0) begin
			pcout = 0;
		end else if (PCWre) begin
			if (PCSrc == 2'b00) begin
				pcout = pcout + 4;
			end else if (PCSrc == 2'b01) begin
				pcout = pcout + 4 + immediate * 4;
			end else if (PCSrc == 2'b10) begin
				pcout = readData1;
			end else if (PCSrc == 2'b11) begin
				pcout = addr;
			end
		end
	end
endmodule

module alu(
    input [31:0] AData, BData, immediate,
	input [4:0] sa,
	input [2:0] ALUOp,
	input ALUSrcA,ALUSrcB,
	output reg zero,
	output reg [31:0] ALUResult
    );
	initial 
		begin
			ALUResult=0;
			zero=1;
		end
	wire [31:0] A,B;
	assign A=ALUSrcA ? {{27{0}},sa[4:0]} : AData;
	assign B=ALUSrcB ? immediate : BData;
	
	always@(ALUOp or AData or BData or A or B)
		begin
			case(ALUOp)
				3'b000 : ALUResult = A+B;
				3'b001 : ALUResult = A-B;
				3'b010 : ALUResult = (A<B)? 1:0;
				3'b011 : 
				begin
				        if(A<B&&((A[31]==0&&B[31]==0)||(A[31]==1&&B[31]==1))) ALUResult =1;
				        else if(A[31]==0&&B[31]==1) ALUResult=0;
				        else if (A[31]==1 &&B[31]==0) ALUResult=1;
				        else ALUResult=0;
				end
				3'b100 : ALUResult = B << A;
				3'b101 : ALUResult = A | B;
				3'b110 : ALUResult = A & B;
				3'b111 : ALUResult = (-A & B) | (A & -B);
				default : ALUResult = 0;
			endcase
			zero = ALUResult ? 0 : 1;
		end
endmodule

module extend(
    input ExtSel,
	input [15:0] halfImm,
	output reg [31:0] Imm
    );
	initial
		begin
			Imm=0;
		end
	
	always@(ExtSel or halfImm)
		begin
			if(ExtSel)
				Imm={{16{halfImm[15]}},halfImm[15:0]};
			else
				Imm={{16{0}},halfImm[15:0]};
		end

endmodule

module IR(
    input IRWre,clk,
	input [31:0] instruction,
	output reg [31:0] IR_out
    );
	initial
		begin
			IR_out=0;
		end
		
	always@(negedge clk)
	#2
		begin
			if(IRWre)
				begin
					IR_out <= instruction;
				end
		end
endmodule

module DataLate(
    input [31:0] i_data,
	input clk,
	output reg [31:0] o_data
    );
	always@(negedge clk)
		begin
			o_data=i_data;
		end
endmodule


module DataMemory(
   input [31:0] DAddr,DataIn,
	input RD,WR,
	output reg [31:0] DataOut
    );
	 
	reg [7:0] memory[0:127];
	integer i;
	
	initial
		begin
			DataOut=0;
			for(i=0;i<128;i=i+1) memory[i] <= 0;
		end
		
	always@(RD or WR or DAddr or DataIn)
		begin
			if(RD)
				begin
					DataOut[31:24]=memory[DAddr];
					DataOut[23:16]=memory[DAddr + 1];
					DataOut[15:8]=memory[DAddr + 2];
					DataOut[7:0]=memory[DAddr + 3];
				end
			else 
					if(WR)
						begin
							memory[DAddr]=DataIn[31:24];
							memory[DAddr + 1]=DataIn[23:16];
							memory[DAddr + 2]=DataIn[15:8];
							memory[DAddr + 3]=DataIn[7:0];
						end
		end

endmodule

module InstructionMemory(
   input [31:0]IAddr,
	input InsMemRW,
	output reg [31:0] instruction
    );
	reg [7:0] mem [0:127];
	initial
		begin
			$readmemb("C:/Users/wonggwan/Desktop/ins.txt",mem);
		end
	always@(IAddr or InsMemRW)
		begin
			if(InsMemRW)
				begin
					instruction[31:24]=mem[IAddr];
					instruction[23:16]=mem[IAddr+1];
					instruction[15:8]=mem[IAddr+2];
					instruction[7:0]=mem[IAddr+3];
				end
		end
endmodule

module RegisterFile(
    input [4:0] rs,rt,rd,
	input [31:0] WriteData,
	input [1:0] RegDst,
	input clk,RegWre,
	output reg [31:0] ReadData1,ReadData2
    );
	reg [31:0] register[0:31];
	reg [4:0] WriteReg;
	integer i;
	
	initial
		begin
			for(i=0;i<32;i=i+1) register[i]=0;
		end
		
	always@(negedge clk)
		begin
			case(RegDst)
				2'b00:WriteReg = 5'b11111;
				2'b01:WriteReg = rt;
				2'b10:WriteReg = rd;
			endcase
			assign ReadData1=register[rs];
			assign ReadData2=register[rt];
		end
	always@(posedge clk)
		begin
			if(WriteReg != 0 && RegWre) register[WriteReg] = WriteData;
		end
endmodule

module pcaddr(
   input [25:0] in,
	input [31:0] PC0,
	output reg [31:0] out
    );
	wire [31:0] PC4;
	assign PC4=PC0+4;
	always@(*)
		begin
			out[31:28]=PC4[31:28];
			out[27:2]=in[25:0];
			out[1:0]=0;
		end
endmodule

module ControlUnit(
   input clk,zero,rst,
	input [5:0] op,
	output reg WrRegData, ExtSel, RegWre, ALUSrcB, ALUSrcA, InsMemRW, IRWre, RD, WR, ALUM2Reg, PCWre,
	output reg[2:0] ALUOp,
	output reg[1:0] RegDst, PCSrc
    );
	
	parameter[2:0] If=3'b000, id=3'b001,
		exe1=3'b110,
		exe2=3'b101,
		exe3=3'b010,
		wb1=3'b111,
		wb2=3'b100,
		mem=3'b011;
	
	parameter[5:0] addu=6'b000000, subu=6'b000001, addiu=6'b000010,
		Or=6'b010000, And=6'b010001, ori=6'b010010, sll=6'b011000,
		sltu=6'b100110, slt=6'b100111, sw=6'b110000, lw=6'b110001,
		beq=6'b110100, j=6'b111000, jr=6'b111001, jal=6'b111010, halt=6'b111111;
	
	reg [2:0] state,nextstate;
	
	initial
		begin
			PCWre=1;
			ALUSrcA=0;
			ALUSrcB=0;
			ALUM2Reg=0;
			RegWre=0;
			WrRegData=0;
			InsMemRW=0;
			RD=0;
			WR=0;
			IRWre=1;
			ExtSel=1;
			PCSrc=2'b00;
			RegDst=2'b01;
			ALUOp=3'b000;
			state=If;
			nextstate=state;
		end
	
	always@(posedge clk)
		begin
			if(rst==0)
				state<=If;
			else
				state<=nextstate;
		end
		
	always@(state or op)
		begin
			case(state)
				If:nextstate=id;
				id:begin
						case(op)
							beq:nextstate=exe2;
							sw:nextstate=exe3;
							lw:nextstate=exe3;
							j:nextstate=If;
							jal:nextstate=wb1;
							jr:nextstate=If;
							halt:nextstate=If;
							addu:nextstate=exe1;
							subu:nextstate=exe1;
							addiu:nextstate=exe1;
							Or:nextstate=exe1;
							And:nextstate=exe1;
							ori:nextstate=exe1;
							sll:nextstate=exe1;
							sltu:nextstate=exe1;
							slt:nextstate=exe1;
						endcase
					end
				exe1:nextstate=wb1;
				exe2:nextstate=If;
				exe3:nextstate=mem;
				mem:if(op==lw) nextstate=wb2;
					 else if(op==sw) nextstate=If;
				wb1:nextstate=If;
				wb2:nextstate=If;
			endcase
		end
		
		always@(state)
		//if(op!=6'b111110)
		//begin
			begin
				if(state==If && op != halt) PCWre=1;
				else PCWre=0;
				
				if(op==sll) ALUSrcA=1;
				else ALUSrcA=0;
				
				if(op==addiu||op==ori||op==lw||op==sw) ALUSrcB=1;
				else ALUSrcB=0;
				
				if(op==lw) ALUM2Reg=1;
				else ALUM2Reg=0;
				
				if(state==wb1||state==wb2||op==jal) RegWre=1;
				else RegWre=0;
				
				if(op==jal) WrRegData=0;
				else WrRegData=1;
				
				InsMemRW=1;
				
				if(state==mem && op==sw)
					begin
						WR=1;
						RD=0;
					end
				else
					begin
						WR=0;
						RD=1;
					end
				
				if(op !=halt) IRWre=1;
				else IRWre=0;
				
				if(op==ori) ExtSel=0;
				else ExtSel=1;
				
				case(op)
					j:PCSrc=2'b11;
					jal:PCSrc=2'b11;
					jr:PCSrc=2'b10;
					beq:if(zero) PCSrc=2'b01;
						 else PCSrc=2'b00;
					default:PCSrc=2'b00;
				endcase
				
				if(op==jal) RegDst=2'b00;
				else if(op==addiu||op==ori||op==lw) RegDst=2'b01;
				else if(op==addu||op==subu||op==Or||op==And||op==sltu||op==sll||op==slt) RegDst=2'b10;
			
				case(op)
					subu:ALUOp=3'b001;
					Or:ALUOp=3'b101;
					And:ALUOp=3'b110;
					ori:ALUOp=3'b101;
					sll:ALUOp=3'b100;
					sltu:ALUOp=3'b010;
					beq:ALUOp=3'b001;
					slt:ALUOp = 3'b011;
					default: ALUOp=3'b000;
				endcase
				
				if(state==If)
					begin
						RegWre=0;
						WR=0;
					end
			end
endmodule

module multicpu(
    input clk,rst
    );
    wire PCWre,InsMemRW,IRWre,WrRegDSrc,ExtSel,RegWre,ALUSrcA,ALUSrcB,RD,WR,ALUM2Reg,zero;
        wire [1:0] RegDst,PCSrc;
        wire [2:0] ALUOp,state;
        wire [4:0] rs,rt,rd,sa;
        wire [5:0] op;
        wire [15:0] halfImmediate;
        wire [25:0] addr26;
        wire [31:0] IAddr,IR_in,instruction,immediate,RegData1,RegData2,AData,BData,DataIn,DAddr,DataOut,result,ALUM2DR_in,ALUM2DR_out,PC4,WriteData,alu_a,alu_b,addr32;
        
        assign PC4=IAddr+4;
        assign op=instruction[31:26];
        assign rs=instruction[25:21];
        assign rt=instruction[20:16];
        assign rd=instruction[15:11];
        assign sa=instruction[15:11];
        assign addr26=instruction[25:0];
        assign halfImmediate=instruction[15:0];
        assign WriteData=WrRegDSrc ? ALUM2DR_out:PC4;
        assign ALUM2DR_in=ALUM2Reg? DataOut:result;
        assign DataIn=BData;
        
        pc pc(clk,rst,PCWre,PCSrc,immediate,RegData1,addr32,IAddr);
        alu alu(AData,BData,immediate,sa,ALUOp,ALUSrcA,ALUSrcB,zero,result);
        extend extend(ExtSel,halfImmediate,immediate);
        IR ir(IRWre,clk,IR_in,instruction);
        DataLate ADR(RegData1,clk,AData);
        DataLate BDR(RegData2,clk,BData);
        DataLate ALUM2DR(ALUM2DR_in,clk,ALUM2DR_out);
        DataLate ALUout(result,clk,DAddr);
        DataMemory DataMemory(DAddr,DataIn,RD,WR,DataOut);
        InstructionMemory InsMemory(IAddr,InsMemRW,IR_in);
        RegisterFile register(rs,rt,rd,WriteData,RegDst,clk,RegWre,RegData1,RegData2);
        pcaddr pcaddr(addr26,IAddr,addr32);
        ControlUnit ControlUnit(clk,zero,rst,op,WrRegDSrc, ExtSel, RegWre, ALUSrcB, ALUSrcA, InsMemRW, IRWre, RD, WR, ALUM2Reg, PCWre,ALUOp,RegDst, PCSrc);

endmodule
