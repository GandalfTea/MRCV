
#ifndef MRCV_SYMBOLIC_CORE
#define MRCV_SYMBOLIC_CORE

namespace MRCV {

template <typename p, typename r>
struct GainSymbol {
	GainSymbol();

	double eval(p P, r R) {
		return p / (p + r);
	}	
}
				


} // namespace
#endif
