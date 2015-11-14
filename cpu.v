`timescale 1ps/1ps

/* 
// A simple pipelined implementation of p4
//
// It has six stages: F, D, R, X, L, W
//
// There is a feedback network that allows any stage
// to flush the pipeline by setting
//
//    x_flush = 1
//    x_target = where to start fetching from
//
// Later stages have precedence over eariler stages.
//
// Three stages generate their own flush signals in the current implementation:
//
//    D for jmp instructions
//    X for jeq instructions
//    W for halt instructions
//
// Stalling is implemented using a per register counter. The counter
// gets set to the number of cycles to stall in order to find the correct
// value in the register.
//
// We only stall for RAW dependencies
//
// No forwarding, no prediction

*/



module main();

    initial begin
        $dumpfile("cpu.vcd");
        $dumpvars(1,main);
        $dumpvars(1,rf);
    end


    /////////////
    // Testing //
    /////////////
    wire [15:0]r0 = rf.data[0];
    wire [15:0]r1 = rf.data[1];
    wire [15:0]r2 = rf.data[2];
    wire [15:0]r3 = rf.data[3];
    wire [15:0]r4 = rf.data[4];
    wire [15:0]r5 = rf.data[5];
    wire [15:0]r6 = rf.data[6];
    wire [15:0]r7 = rf.data[7];
    wire [15:0]r15 = rf.data[15];

    reg [15:0]steps = 0;

    // clock
    wire clk;
    clock c0(clk);
    input [15:0]cycle;
    reg [15:0]inc = 0;
    counter ctr(W_halt,clk,W_v | D_isJump | R_flush,cycle);

    // Functions generate combinational logic

    /*
    Get rb | ra wire value
    */
    function [7:0]getII;
        input [15:0]inst;
        getII = inst[11:4];
    endfunction
    /*
    Get ra | rt wire value
    */
    function [7:0]getSS;
        input [15:0]inst;
        getSS = inst[7:0];
    endfunction
    /*
    Get ra wire value
    */
    function [3:0]RA;
        input [15:0]inst;
        RA = inst[11:8];
    endfunction

    /*
    Get rb wire value
    */
    function [3:0]RB;
        input [15:0]inst;
        RB = inst[7:4];
    endfunction

    /*
    Get rt wire value
    */
    function [3:0]RT;
        input [15:0]inst;
        RT = inst[3:0];
    endfunction
 
    function isMov;
        input v;
        input [15:0]inst;
        isMov = v & (inst[15:12] == 0);
    endfunction
 
    function isAdd;
        input v;
        input [15:0]inst;
        isAdd = v & (inst[15:12] == 1);
    endfunction

    function isJmp;
        input v;
        input [15:0]inst;
        isJmp = v & (inst[15:12] == 2);
    endfunction

    function isHalt;
        input v;
        input [15:0]inst;
        isHalt = v & (inst[15:12] == 3);
    endfunction

    function isLd;
        input v;
        input [15:0]inst;
        isLd = v & (inst[15:12] == 4);
    endfunction

    function isLdr;
        input v;
        input [15:0]inst;
        isLdr = v & (inst[15:12] == 5);
    endfunction

    function isJeq;
        input v;
        input [15:0]inst;
        isJeq = v & (inst[15:12] == 6);
    endfunction

    function isSt; 
        input v;
        input [15:0]inst;
        isSt = v & (inst[15:12] == 7);
    endfunction

    /*
    Are we adding | loading | load jumping?
    */
    function usesRA;
        input v;
        input [15:0]inst;
        usesRA = isAdd(v,inst) | isLdr(v,inst) | isJeq(v,inst) | isSt(v,inst);
    endfunction

    /*
    Are we adding | loading | load jumping?
    */
    function usesRB;
        input v;
        input [15:0]inst;
        usesRB = isAdd(v,inst) | isLdr(v,inst) | isJeq(v,inst);
    endfunction

    /*
    Are we adding | loading | load jumping | moving?
    */
    function usesRT;
        input v;
        input [15:0]inst;
        usesRT = isAdd(v,inst) | isLd(v,inst) | isLdr(v,inst) | isMov(v,inst) | isJeq(v,inst);
    endfunction
     
    // Compute the per-register stall counter
    function[3:0]count;
        input [3:0]index;
        count = ((~D_stall) & ~(D_flush) & usesRT(D_v,D_inst) & (RT(D_inst) == index)) ? 5 :
                (counters[index] == 0) ? 0 : (counters[index] -1);
    endfunction

    // An array of per register counters, each representing the number
    // of cycles remaining until the register is ready
    reg [3:0]counters[0:15];
      
    //wire X_vavbUndefined = (^X_va === 1'bx | ^X_vb === 1'bx);
    wire D_jumpNotTaken = (D_isJumpEqual & jeqStall==1 & D_stall==1 & X_vb != X_va);
    wire D_jumpTaken =  (D_isJumpEqual & jeqStall==1 & D_stall==1 & X_vb == X_va);
 
    // next PC value
    wire [15:0]nextPC = D_jumpNotTaken ? X_savedPC :
                        D_jumpTaken  ? D_jumpPC :
                        D_flush ?  D_target : //jump to old value
                        D_stall ?  F_pc :     //jump to itself
                        F_v ?      (F_pc + 1) :   //increment
                        0;
 
    wire [15:0]memInLoad =  isLdr(X_v,X_inst) ?  X_va + X_vb :     
                            isLd(D_v,D_inst) ?   getII(D_inst) :
                            16'hxxxx;

    wire [15:0]memOutLoad;

    wire memEnable = isSt(W_v,W_inst); 
    wire [15:0]memWrite =  {8'h0, getSS(W_inst)};
      
    // memory output
    wire [15:0]memOut;
    wire [7:0]getSSWire = getSS(W_inst); 
    wire [15:0]memWriteValue = (memEnable) ? W_va : 16'hxxxx;
 
    mem i0(clk,
        //read port 1
        nextPC,memOut,
        //read port 2
        memInLoad,memOutLoad,
        //write port
        memEnable,
        memWrite, 
        W_va
        );  

    ///////////
    // Fetch //
    ///////////

    reg F_v = 0;                     // Do we have a valid instruction
    reg [15:0] F_pc;                 // PC

    ////////////
    // Decode //
    ////////////

    reg D_v = 0;
    reg [15:0] D_pc;

    // The decode stage implements stalling by feeding its state
    // back to itself and forcing the fetch stage to do the same
    // We save the instruction read from memory in the D_savedInst
    // register and use it in the following cycle instead of loading
    // it from memory again

    reg [15:0] D_savedInst;       // saved instruction
    reg [15:0] D_savedPC;       // saved pc  
    reg [15:0] R_savedPC;       // saved pc 
    reg [15:0] X_savedPC;       // saved pc  

    reg [15:0] D_jumpPC;       // saved pc 
    reg D_useSaved = 0;           // 1 if using stalled instruction
    wire [15:0] D_inst = validJump ? W_loadRegister : 
                         D_useSaved ? D_savedInst : 
                         memOut;
 
    reg [15:0]validJump = 0; 
    wire [15:0]getSSW = getSS(W_instPass1);

    reg [15:0] W_instPass1; 
    reg [15:0] W_instPass2;                     
    always @(posedge clk) begin

        if(getSSW+1 == nextPC) 
            validJump <= 1;
        else 
            validJump <= 0;
         
        W_instPass1 <= W_inst;
        W_instPass2 <= W_instPass1;
    end 
 
    wire D_isJump = isJmp(D_v,D_inst);
    wire D_isJumpEqual = isJeq(D_v,D_inst); 
    reg R_isJumpEqual;
    reg X_isJumpEqual;
    reg L_isJumpEqual;

    always @(posedge clk) begin

        R_isJumpEqual <= D_isJumpEqual;
        X_isJumpEqual <= R_isJumpEqual;
        L_isJumpEqual <= X_isJumpEqual;

        if(D_isJumpEqual) begin  

            D_jumpPC <= D_pc + RTDXinst;
            D_savedPC  <= F_pc;

            R_savedPC  <= D_savedPC;
            X_savedPC  <= R_savedPC; 
 
        end  
    end
 
    wire D_flush = R_flush | D_isJump | D_isJumpEqual ;  
    wire [15:0]D_target = R_flush ? R_target : 
                          D_isJump ? D_inst[11:0] : 
                          16'hxxxx;

    reg[15:0] jeqStall = 0;                       
    /*
    Do we need to stall?
    We stall if bitmask at the decode ra|rb > 0 (meaning those are being updated)
    and the current decode instruction uses these registers
    */  


 
    wire stallAddForwarding = (R_forwardAdd1A2B | R_forwardAdd2A1B | R_forwardAdd2A2A | R_forwardAdd1B1B) |  //move add forward
                              (R_forwardAddAdd1B1B);   //add add forward

    //  !stallAddForwarding & //forward add [0000 0011 1002]

    wire D_stall = !stallAddForwarding & 
                   ((usesRA(D_v,D_inst) & (counters[RA(D_inst)] != 0)) |
                   (usesRB(D_v,D_inst) & (counters[RB(D_inst)] != 0)) | 
                   (jeqStall > 0 | isJeq(D_v,D_inst)));
 
    always @(posedge clk) begin
 
        if(isJeq(D_v,D_inst))
            jeqStall <= 6;
        if(jeqStall != 0)
            jeqStall <= jeqStall-1;
 
    end

    ///////////////
    // Registers //
    ///////////////

    reg R_v = 0;
    reg [15:0]R_pc;
    reg [15:0]R_inst;
    wire R_flush = X_flush;
    wire [15:0]R_target = X_target;
    wire write = isAdd(W_v,W_inst) | isLd(W_v,W_inst) | isLdr(W_v,W_inst) | isMov(W_v,W_inst) | W_forwardAdd_v | W_forwardAddAdd_v; //regfor
 
    reg [15:0]W_va;
    reg [15:0]W_vb;
    reg [15:0]L_va;
    reg [15:0]L_vb;
    regs rf(clk,
        1,     RA(D_inst), X_va, 
        1,     RB(D_inst), X_vb, 
        write, RT(W_inst), W_output);
    
    always @(posedge clk) begin
   
        L_va <= X_va;
        L_vb <= X_vb;
        W_va <= L_va;
        W_vb <= L_vb;
 
    end

    wire [3:0] RAXinst = RA(X_inst);
    wire [3:0] RBXinst = RB(X_inst);
    wire [3:0] RTDXinst = RT(D_inst);

    /////////////
    // eXecute //
    /////////////

    reg X_v = 0;
    reg [15:0]X_pc;
    reg [15:0]X_inst;
    wire [15:0]X_va;
    wire [15:0]X_vb;
    // Compute the result
  
    wire isLdrWire = isLdr(X_v,X_inst);
    wire isAddWire = isAdd(X_v,X_inst);
    wire isLdWire  = isLd(X_v,X_inst);
    wire isMovWire = isMov(X_v,X_inst);
    wire isJeqWire = isJeq(X_v,X_inst);
    wire [7:0]getIIW   = getII(D_inst);
 

    wire [15:0]X_res = isLdr(X_v,X_inst) ? memOutLoad :
                       isAdd(X_v,X_inst) ? X_va + X_vb :
                       (isMov(X_v,X_inst) | isLd(X_v,X_inst)) ? X_inst[11:4] :
                       16'hxxxx;
    //    Do we need to flush the pipeline?
    //    if later stages want to, we let them do it
    //    if we have a taken jeq, we do it
    wire X_flush = L_flush | (isJeq(X_v,X_inst) & (X_va == X_vb));  
    wire [15:0]X_target = L_flush ? L_target : (X_pc + X_inst[3:0]);// THIS PART SETS NEXT TAZRGET AND NEXT PC
  
    //////////
    // Load //
    //////////

    reg L_v = 0;
    reg [15:0]L_pc;
    reg [15:0]L_inst;
    reg [15:0]L_res;
    wire L_flush = W_flush;
    wire [15:0]L_target = W_target;

    ////////////////
    // Write-back //
    ////////////////

    reg W_v = 0;
    reg [15:0]W_pc;
    reg [15:0]W_inst;
    wire [15:0]W_memout = memOutLoad;
    reg [15:0]W_res;

    /*
    Registers that keep load value
    */
    reg [15:0]L_loadRegister;
    reg [15:0]W_loadRegister;
    reg [15:0]loadAddRegister;
    wire isLdWireW = isLd(W_v,W_inst) ;
    wire isLdrWireW = isLdr(W_v,W_inst) ;


    // The output comes from either memory or the X result depending
    // on the instruction

    reg [15:0]W_forwardAddAdd_ALU = 16'hxxxx;
    always @(posedge clk) begin
        W_forwardAddAdd_ALU <= W_output + W_output;
    end
    wire [15:0]W_output = W_forwardAddAdd_v ? W_forwardAddAdd_ALU : 
                          W_forwardAdd_v ? W_forwardAdd_ALU : //regfor
                          isLd(W_v,W_inst)  ?  W_loadRegister :   
                          isLdr(W_v,W_inst) ?  memOutLoad :
                                               W_res;
    wire W_halt = isHalt(W_v,W_inst);
    // Flush the pipeline if we have a halt
    wire [15:0]W_target = W_pc;
    wire W_flush = W_halt;


    reg[15:0] counter1;
    reg[15:0] counter2;
    reg[15:0] counter3;
    reg[15:0] counter4;
    reg[15:0] counter5;
    reg[15:0] counter6;
    reg[15:0] counter7;
    reg[15:0] counter8;
    reg[15:0] counter9;
    reg[15:0] counter10;
    reg[15:0] counter11;
    reg[15:0] counter12;
    reg[15:0] counter13;
    reg[15:0] counter14;
    reg[15:0] counter15; 

    ////////////////
    // Forwarding //
    ////////////////
 

    //regfor

    //0011 0000 1012
    wire R_forwardAdd1A2B = isAdd(D_v,D_inst) & isMov(X_v,X_inst) & isMov(R_v,R_inst) & RA(D_inst)==RT(R_inst) & RB(D_inst)==RT(X_inst);
    //0000 0011 1012
    wire R_forwardAdd2A1B = isAdd(D_v,D_inst) & isMov(X_v,X_inst) & isMov(R_v,R_inst) & RA(D_inst)==RT(X_inst) & RB(D_inst)==RT(R_inst);
    //0000 0001 1112 and no 1*** inbetween
    wire R_forwardAdd2A2A = isAdd(D_v,D_inst) & isMov(X_v,X_inst) & RA(D_inst)==RT(X_inst) & RB(D_inst)==RT(X_inst);
    //0000 0001 1002
    wire R_forwardAdd1B1B = isAdd(D_v,D_inst) & isMov(R_v,R_inst) & RA(D_inst)==RT(R_inst) & RB(D_inst)==RT(R_inst);


    //0011 0000 1012 1223
    wire R_forwardAddAdd2A2A = isAdd(D_v,D_inst) & isAdd(X_v,X_inst) & RA(D_inst)==RT(X_inst) & RB(D_inst)==RT(X_inst);
    //0011 1012 0000 1223
    wire R_forwardAddAdd1B1B = isAdd(D_v,D_inst) & isAdd(R_v,R_inst) & RA(D_inst)==RT(R_inst) & RB(D_inst)==RT(R_inst);

    reg [15:0] R_forwardAdd_ALU = 16'hxxxx;
    reg [15:0] X_forwardAdd_ALU = 16'hxxxx;
    reg [15:0] L_forwardAdd_ALU = 16'hxxxx;
    reg [15:0] W_forwardAdd_ALU = 16'hxxxx;








    //last issue for add add forwarding-we add twice. output twice too long. 





    //ADD ADD FORWARDING
   /*
    reg R_forwardAddAdd_v = 16'hxxxx;
    reg X_forwardAddAdd_v = 16'hxxxx;
    reg L_forwardAddAdd_v = 16'hxxxx;
    reg W_forwardAddAdd_v= 16'hxxxx;

   */
    reg R_forwardAddAdd_v = 0;
    reg X_forwardAddAdd_v = 0;
    reg L_forwardAddAdd_v = 0;
    reg W_forwardAddAdd_v = 0;

    reg R_forwardAdd_v = 0;
    reg X_forwardAdd_v = 0;
    reg L_forwardAdd_v = 0;
    reg W_forwardAdd_v = 0;
  
    //forwarding addition
    always @(posedge clk) begin

        if(R_forwardAddAdd1B1B) begin //add add forward  
            R_forwardAddAdd_v <= 1;
        end else if(R_forwardAdd1B1B) begin
            R_forwardAdd_ALU <= (getII(R_inst) + getII(R_inst));
            R_forwardAdd_v <= 1; 
        end else if(R_forwardAdd2A2A) begin
            R_forwardAdd_ALU <= (getII(X_inst) + getII(X_inst));
            R_forwardAdd_v <= 1; 
        end else if(R_forwardAdd1A2B | R_forwardAdd2A1B) begin
            R_forwardAdd_ALU <= (getII(X_inst) + getII(R_inst));
            R_forwardAdd_v <= 1;
        end else begin 
            R_forwardAdd_ALU <= 16'hxxxx;
            R_forwardAdd_v <= 0; 
            R_forwardAddAdd_v <= 0;
        end
        
        X_forwardAdd_ALU <= R_forwardAdd_ALU;
        L_forwardAdd_ALU <= X_forwardAdd_ALU;
        W_forwardAdd_ALU <= L_forwardAdd_ALU;

        X_forwardAdd_v <= R_forwardAdd_v;
        L_forwardAdd_v <= X_forwardAdd_v;
        W_forwardAdd_v <= L_forwardAdd_v;


        X_forwardAddAdd_v <= R_forwardAddAdd_v;
        L_forwardAddAdd_v <= X_forwardAddAdd_v;
        W_forwardAddAdd_v <= L_forwardAddAdd_v;
    end
 


    // Sequential logic, update all flip-flops
    always @(posedge clk) begin

        inc <= inc+1;
        steps <= steps+1;
  
        /*
        Save load register values
        */
        if(isLd(X_v,X_inst))
            L_loadRegister <= memOutLoad;

        //forwarded load after 7*** instruction
        if(isLd(L_v,L_inst) & isSt(W_v,W_inst) & getSS(W_inst)==getII(L_inst))
            W_loadRegister <= W_va;
        else 
            W_loadRegister <= L_loadRegister;
 

        if(isLdr(X_v,X_inst))
            loadAddRegister <= X_va + X_vb;

        if(W_halt)
            $finish;

        F_v <= 1;
        F_pc <= nextPC;

        D_v <= D_flush ? 0 : (F_v  | D_stall);
        D_pc <= D_stall ? D_pc : F_pc;
        D_useSaved <= D_stall;
        D_savedInst <= D_inst;

        R_v <= D_v & (~D_flush) & (~D_stall);
        R_pc <= D_pc;
        R_inst <= D_inst;

        X_v <= R_v & (~R_flush);
        X_pc <= R_pc;
        X_inst <= R_inst;

        L_v <= X_v & (~X_flush);
        L_pc <= X_pc;
        L_inst <= X_inst;
        L_res <= X_res;

        W_v <= L_v & (~L_flush);
        W_pc <= L_pc;
        W_inst <= L_inst;
        W_res <= L_res;

        // counters
        if (F_v) begin
  
            counter1 <= count(0); 
            counter2 <= count(1); 
            counter3 <= count(2); 
            counter4 <= count(3); 
            counter5 <= count(4); 
            counter6 <= count(5); 
            counter7 <= count(6); 
            counter8 <= count(7); 
            counter9 <= count(8); 
            counter10 <= count(9); 
            counter11 <= count(10); 
            counter12 <= count(11); 
            counter13 <= count(12); 
            counter14 <= count(13); 
            counter15 <= count(14); 


            counters[0] <= count(0);
            counters[1] <= count(1);
            counters[2] <= count(2);
            counters[3] <= count(3);
            counters[4] <= count(4);
            counters[5] <= count(5);
            counters[6] <= count(6);
            counters[7] <= count(7);
            counters[8] <= count(8);
            counters[9] <= count(9);
            counters[10] <= count(10);
            counters[11] <= count(11);
            counters[12] <= count(12);
            counters[13] <= count(13);
            counters[14] <= count(14);
            counters[15] <= count(15);
        end else begin
            counters[0] <= 0;
            counters[1] <= 0;
            counters[2] <= 0;
            counters[3] <= 0;
            counters[4] <= 0;
            counters[5] <= 0;
            counters[6] <= 0;
            counters[7] <= 0;
            counters[8] <= 0;
            counters[9] <= 0;
            counters[10] <= 0;
            counters[11] <= 0;
            counters[12] <= 0;
            counters[13] <= 0;
            counters[14] <= 0;
            counters[15] <= 0;
        end
 
    end
 
endmodule
