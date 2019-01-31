
/* FMT_ENTRY( idx_, exp_(0/1 only), exp_name_, exp_id_, name_, type_id_, 
 *            sign_bit_, float_bit_, pbits_, bits_, fbits_, ldiv_bits_)
 * Specifications:
   - idx_ (8bits) : is an index that increased from 0 to N where N are the total 
      number of formats
   - exp_ (1bit) : it is a boolean flag (0 or 1) indicating whether the format is 
      available as a public APIs ai_buffer format. in this case the field 
      exp_name_ indicates the enum name of the ai_buffer format 
   - exp_id_ (8 bits):  field could be any number in the range [0x0..0xff] 
      thus  
   - name_   : it is the enum used to define the ai_data_format. 
   - type_id_ (4bits) : it is used to define the "family" of the format: 
      see @ref AI_FMT_I as an example. Currently supported types are: 
      AI_FMT_I (integer types), AI_FMT_Q (fixed point types), AI_FMT_FLOAT
      (floating point values), AI_FMT_LUT4 or AI_FMT_LUT8 (compressed formats)
   - sign_bit_ (1bit) : codes whether or not the format is of a signed type
   - float_bit_ (1bit) : codes if the format is float
   - pbits_ (4bits) : number of padding bits for the format
   - bits_ (7bits)  : size in bits of the format (NB: integer+fractional bits)
   - fbits_ (7bits) : number of fractional bits for the format (for AI_FMT_Q only)
   - ldiv_bits (4 bits) : right shift value for computing the byte size of the format
   
  */

/* Macro tricks are here:
 * https://github.com/pfultz2/Cloak/wiki/C-Preprocessor-tricks,-tips,-and-idioms
 */

/* Format none entry */
FMT_ENTRY( 0, 1, AI_BUFFER_FORMAT_NONE,    0x00, NONE, AI_FMT_I, 0, 0, 0, 0, 0, 0)

/* Floating point formats */
FMT_ENTRY( 1, 1, AI_BUFFER_FORMAT_FLOAT,   0x01, FLOAT,  AI_FMT_FLOAT, 1, 1, 0, 32,  0, 0)
FMT_ENTRY( 2, 0, AI_BUFFER_FORMAT_FLOAT64, 0x02, FLOAT64,  AI_FMT_FLOAT, 1, 1, 0, 64,  0, 0)

/* Integer formats */
FMT_ENTRY( 3, 1, AI_BUFFER_FORMAT_U8,  0x10, U8,  AI_FMT_I, 0, 0, 0, 8,  0, 0)
FMT_ENTRY( 4, 0, AI_BUFFER_FORMAT_U16, 0x11, U16, AI_FMT_I, 0, 0, 0, 16, 0, 0)
FMT_ENTRY( 5, 0, AI_BUFFER_FORMAT_U32, 0x12, U32, AI_FMT_I, 0, 0, 0, 32, 0, 0)
FMT_ENTRY( 6, 0, AI_BUFFER_FORMAT_U64, 0x13, U64, AI_FMT_I, 0, 0, 0, 64, 0, 0)
FMT_ENTRY( 7, 0, AI_BUFFER_FORMAT_U4,  0x14, U4,  AI_FMT_I, 0, 0, 0, 4,  0, 0)
FMT_ENTRY( 8, 0, AI_BUFFER_FORMAT_S8,  0x20, S8,  AI_FMT_I, 1, 0, 0, 8,  0, 0)
FMT_ENTRY( 9, 0, AI_BUFFER_FORMAT_S16, 0x21, S16, AI_FMT_I, 1, 0, 0, 16, 0, 0)
FMT_ENTRY(10, 0, AI_BUFFER_FORMAT_S32, 0x22, S32, AI_FMT_I, 1, 0, 0, 32, 0, 0)
FMT_ENTRY(11, 0, AI_BUFFER_FORMAT_S64, 0x23, S64, AI_FMT_I, 1, 0, 0, 64, 0, 0)

/* Fixed-point formats including ARM CMSIS Q7, Q15, Q31 ones */
FMT_ENTRY(12, 0, AI_BUFFER_FORMAT_Q,    0x30, Q,    AI_FMT_Q, 1, 0, 0, 0,   0, 0)
FMT_ENTRY(13, 1, AI_BUFFER_FORMAT_Q7,   0x31, Q7,   AI_FMT_Q, 1, 0, 0, 8,   7, 0)
FMT_ENTRY(14, 1, AI_BUFFER_FORMAT_Q15,  0x32, Q15,  AI_FMT_Q, 1, 0, 0, 16, 15, 0)
FMT_ENTRY(15, 0, AI_BUFFER_FORMAT_Q31,  0x33, Q31,  AI_FMT_Q, 1, 0, 0, 32, 31, 0)

/* Compressed formats */
FMT_ENTRY(16, 0, AI_BUFFER_FORMAT_LUT4_FLOAT, 0x50, LUT4_FLOAT,  AI_FMT_LUT4, 1, 1, 0, 32, 0, 3)
FMT_ENTRY(17, 0, AI_BUFFER_FORMAT_LUT8_FLOAT, 0x51, LUT8_FLOAT,  AI_FMT_LUT8, 1, 1, 0, 32, 0, 2)
FMT_ENTRY(18, 0, AI_BUFFER_FORMAT_LUT4_Q15, 0x52, LUT4_Q15,  AI_FMT_LUT4, 1, 0, 0, 16, 15, 2)
FMT_ENTRY(19, 0, AI_BUFFER_FORMAT_LUT8_Q15, 0x53, LUT8_Q15,  AI_FMT_LUT8, 1, 0, 0, 16, 15, 1)

#undef FMT_ENTRY
