module sqrt
		#(parameter N = 16)(
		input [N-1:0]radical,
		output [N/2-1:0]resualt,
		output [N/2:0]remainder,    //余数
		output		  done
		);
		parameter k = (N + N%2)/2; //to get the nearest even number of 
		integer i;
		reg done_temp;
		reg [N-1:0] r[0:k];
		reg [N:0] b[0:k];
		always@(radical)
		begin
			done_temp<=0;
			r[0]<=0;
			b[0]<=0;
		for(i=0;i<k;i = i+1)
		begin
			if (4*r[i] + 2*radical[N - 2*i - 1] + radical[N - 2*i - 2] >= 4*b[i] + 1)
			begin
				r[i+1] <= 4*r[i] + 2*radical[N - 2*i  - 1] + radical[N - 2*i - 2] - 4*b[i] - 1;
				b[i+1] <= 2*b[i] + 1;
			end
			else
			begin
				r[i+1] <= 4*r[i] + 2*radical[N - 2*i - 1] + radical[N - 2*i - 2];
				b[i+1] <= 2*b[i];
			end
		end
			if(b[k]!=0)begin
				done_temp<=1'b1;
			end
		end
		assign resualt = b[k];
		assign remainder = r[k];
		assign done = done_temp;
endmodule		