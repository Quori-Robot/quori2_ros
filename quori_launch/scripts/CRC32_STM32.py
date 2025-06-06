
# CRC32/MPEG-2 as implemented by STM32 CRC periperal- data buffer zero padded to multiples of 4 bytes
class CRC32_STM32(object):


    def __init__(self, poly=0x04c11db7, mask = 0 ):

        self.crc32_constant=poly
        self.out_mask=mask


    def calculate(self, input_data=None):
        try:
            is_string = isinstance(input_data, str)
            is_bytes = isinstance(input_data, bytes)

            if not is_string and not is_bytes:
                raise Exception("Please provide a string or a byte sequence as \
                    argument for calculation.")

            crcValue = 0xffffffff
            data_len = len(input_data)
          
            # process data in groups of 4 bytes/32bits) to match STM32 CRC
            for i in range(0,data_len,4):
                
                wordVal = 0
                # pad data with zeroes if less than 4 bytes
                for b in range(4):
                    
                    if(i+3-b < data_len):
                        c = input_data[i+3-b]
                        d = ord(c) if is_string else c
                        wordVal = (wordVal << 8 ) & 0xFFFFFF00 | d
                
                # simple bi-by-bit implementation of CRC
                crcValue = crcValue ^  wordVal
                
        
                for b in range(32):

                    if(crcValue & 0x80000000) :
                       crcValue = ((crcValue << 1)&0xffffffff) ^ self.crc32_constant
                    else :
                       crcValue = ((crcValue << 1)&0xffffffff)
                      
            
                    wordVal=(wordVal<<1) & 0xffffffff


            # Optionaly: Only for CRC-32: When all bytes have been processed, take the
            # one's complement of the obtained CRC value
            crcValue ^= self.out_mask # (or crcValue = ~crcValue)

            return crcValue
        
        except Exception as e:
            print("EXCEPTION(calculate): {}".format(e))

