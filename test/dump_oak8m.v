module dump ();
  initial begin
    $dumpfile("oak8m_test.vcd");
    $dumpvars(0, oak8m);
    #1;
  end
endmodule