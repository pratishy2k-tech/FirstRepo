module tb;

reg clk;

initial begin
clk = 0;
$display("HELLO WORLD");
end


always begin

clk = ~clk ;

end



endmodule 
