namespace Codec2NetFunctions
{
	extern "C" { __declspec(dllexport)  void open(int mode); }

	extern "C" { __declspec(dllexport)  void encode(short pcm_in[], unsigned char* codec_frame); }
	extern "C" { __declspec(dllexport)  void decode(const unsigned char* codec_frame, short pcm_out[]); }

	extern "C" { __declspec(dllexport)  void decode_ber(const unsigned char* codec_frame, short pcm_out[], float ber_est); }
	extern "C" { __declspec(dllexport)  int  samples_per_frame(); }
	extern "C" { __declspec(dllexport)  int  bits_per_frame(); }

	extern "C" { __declspec(dllexport)  void set_lpc_post_filter(int enable, int bass_boost, float beta, float gamma); }
	extern "C" { __declspec(dllexport)  int  get_spare_bit_index(); }
	extern "C" { __declspec(dllexport)  int  rebuild_spare_bit(int unpacked_bits[]); }
	extern "C" { __declspec(dllexport)  void set_natural_or_gray(int gray); }
	extern "C" { __declspec(dllexport)  void set_softdec(float *softdec); }

	extern "C" { __declspec(dllexport)  void close(); }
}