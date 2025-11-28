using System;

public class CRC8Calc {
    private byte[] mTable;
	
    public byte Checksum(byte[] val, int offset, int length ) 
    {
        if(val == null) 
            throw new ArgumentNullException("val");
			
        byte c = 0;

        for(int i = offset; i < offset + length; i++) 
        {
            var b = val[i];
            c = mTable[c ^ b];
        }
    
        return c;
    }
	
    public byte[] GenerateTable(byte polynomial)
    {
        byte[] csTable = new byte[256];
		
        for ( int i = 0; i < 256; ++i ) 
        {
            int curr = i;
			
            for ( int j = 0; j < 8; ++j ) 
            {
                if ((curr & 0x80) != 0) 
                {
                    curr = (curr << 1) ^ (int)polynomial;
                } 
                else 
                {
                    curr <<= 1;
                }
            }
			
            csTable[i] = (byte)curr;
        }
		
        return csTable;
    }
	
    public CRC8Calc(byte polynomial) 
    {
        mTable = new byte[256];
        mTable = GenerateTable(polynomial);
    }
}