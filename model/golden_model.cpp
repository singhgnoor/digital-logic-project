// Bit-accurate C++ golden model of fp_div.v  (Newton-Raphson FP divider + square root)
// Usage: ./golden_model            -> writes vectors.hex, vectors_sqrt.hex and the csv files (see main)
//        ./golden_model --dump SEED ITER AUTO OP out.csv   -> per-vector hw results, same format as the tb results.csv
//        ./golden_model --fault-subset out.csv             -> fault campaign subset of tb_fault.v, same format as fault_results.csv
// Build: g++ -O2 -std=c++17 golden_model.cpp -o golden_model      (do NOT use -ffast-math)
// STAGE 4: seed modes (LUT / line / piecewise linear), convergence monitor (auto_stop),
//          square root (hw_sqrt), fault injection model, more csv outputs for the plots
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>
using namespace std;

static uint32_t f2u(float f){ uint32_t u; memcpy(&u,&f,4); return u; }
static float    u2f(uint32_t u){ float f; memcpy(&f,&u,4); return f; }
static const uint64_t M32 = 0xFFFFFFFFull;

// ---- same 8-entry LUT as seed_lut.v  (x0 = 32/(17+2i) in Q2.30) -------------
static uint32_t lut(int i){ return (uint32_t)llround(32.0/(17+2*i) * (double)(1u<<30)); }

// ---- STAGE 4: same constants as seed_linear.v ---------------------------------
// minimax line on [a,b] for relative error:  c1 = 2/(ab + (a+b)^2/4),  c0 = c1*(a+b)
// c0 in Q2.30, c1 in Q2.14 (16 bit, goes into vedic_16x16)
static void lin_coef(int pwl, int i, uint32_t& c0, uint32_t& c1){
    double a = pwl ? 0.5 + i/16.0 : 0.5, b = pwl ? a + 1/16.0 : 1.0;
    double k1 = 2.0/(a*b + (a+b)*(a+b)/4), k0 = k1*(a+b);
    c0 = (uint32_t)llround(k0 * (double)(1u<<30));
    c1 = (uint32_t)llround(k1 * 16384.0);
}
// x0 = c0 - c1*d16,  d16 = d[30:15] (Q0.16, d[31] is always 0), c1*d16 is Q2.30 directly
// segment index = d[29:27] = mb[22:20] (same as the LUT)
static uint32_t seed_linear(uint32_t d, int pwl){
    uint32_t c0, c1; lin_coef(pwl, (d>>27)&7, c0, c1);
    uint32_t p = c1 * ((d>>15) & 0xFFFF);            // vedic_16x16
    return c0 - p;                                   // sub32
}
static uint32_t seed(uint32_t d, int mode){
    return mode==0 ? lut((d>>27)&7) : seed_linear(d, mode==2);
}

// ---- STAGE 4: same 16 entries as rsqrt_lut.v -----------------------------------
// idx = {ea[0], frac[22:20]},  ea[0]=0 -> x in [0.5,1) (step 1/16),  ea[0]=1 -> x in [0.25,0.5) (step 1/32)
// entry = 2/(sqrt(lo)+sqrt(hi)) in Q2.30
static uint32_t rsq_lut(int idx){
    double lo = (idx&8) ? 0.25 + (idx&7)/32.0 : 0.5 + (idx&7)/16.0;
    double hi = (idx&8) ? lo + 1/32.0 : lo + 1/16.0;
    return (uint32_t)llround(2.0/(sqrt(lo)+sqrt(hi)) * (double)(1u<<30));
}

// ---- STAGE 4: conv_monitor.v : r = e - 1.0 (Q2.30), |r| < 2^-k  <=>  r[31:30-k] all 0s or all 1s ----
static bool small_res(uint32_t e, int k){
    uint32_t r = e - 0x40000000u;
    uint32_t top = r >> (30-k); uint32_t all1 = (1u<<(k+2))-1;
    return top==0 || top==all1;
}

// ---- STAGE 4: fault injection (same meaning as the fi_* regs in fp_div.v) ----
// sel: 0 = x, 1 = e, 2 = d, 3 = mant_A, 4 = prod
// x_k : k=0 seed (written in IDLE), k = value after the k-th iteration
// e_k : e = d*x_k, computed in iteration k (k = 0,1,..)
struct Fault { bool en=false; int sel=0, iter=0, bit=0; };
static uint64_t fmask(const Fault& f, int sel, int k){
    return (f.en && f.sel==sel && (sel>=2 || f.iter==k)) ? (1ull<<f.bit) : 0;
}

struct Cfg { int seed=0; bool autostop=false; int monk=15; Fault fi; };

// ---- the datapath of fp_div.v ------------------------------------------------
struct HwOut { uint32_t result; uint32_t x[9]; uint32_t flags; int iters; bool conv; bool fdet; int cycles; };
                                                                    // x[0]=seed, x[k]=after k-th iteration
                                                                    // flags = {invalid, div_by_zero, overflow, underflow}

HwOut hw_div(uint32_t a, uint32_t b, int n_iter, const Cfg& c = Cfg()){
    HwOut o{}; o.cycles = 1;
    uint32_t sign = (a>>31) ^ (b>>31);
    int ea = (a>>23)&0xFF, eb = (b>>23)&0xFF;

    // ---- STAGE 3: special cases (same priority as fp_div.v), DAZ for subnormal inputs ----
    uint32_t fa = a & 0x7FFFFF, fb = b & 0x7FFFFF;
    bool nana = ea==255 && fa, nanb = eb==255 && fb;
    bool infa = ea==255 && !fa, infb = eb==255 && !fb;
    bool a0 = ea==0, b0 = eb==0;                       // zero or subnormal
    bool r_nan  = nana || nanb || (a0 && b0) || (infa && infb);
    bool r_inf  = !r_nan && (infa || b0);
    bool r_zero = !r_nan && !r_inf && (a0 || infb);
    if(r_nan || r_inf || r_zero){
        bool invalid = (nana && !(fa>>22&1)) || (nanb && !(fb>>22&1)) || (a0 && b0) || (infa && infb);
        bool dbz     = b0 && !a0 && !infa && !nana && !nanb;
        o.flags = (invalid<<3) | (dbz<<2);
        o.result = r_nan ? (nana ? (a | 0x400000u) : nanb ? (b | 0x400000u) : 0x7FC00000u)
                 : r_inf ? (sign<<31) | (0xFFu<<23) : (sign<<31);
        return o;
    }
    const Fault& f = c.fi;
    uint32_t ma = ((1u<<23) | (a & 0x7FFFFF)) ^ (uint32_t)(fmask(f,3,0) & 0xFFFFFF);
    uint32_t mb = (1u<<23) | (b & 0x7FFFFF);
    uint32_t d  = (mb << 7) ^ (uint32_t)fmask(f,2,0);                // Q1.31
    uint32_t x  = seed(mb << 7, c.seed) ^ (uint32_t)fmask(f,0,0);    // Q2.30
    o.x[0] = x;
    int it = 0;
    if(n_iter > 0) for(;;){
        uint32_t ew = (uint32_t)(((uint64_t)d * x) >> 31);         // Q2.30  (~1.0), mul_p[62:31]
        bool conv = small_res(ew, c.monk);                         // monitor sees the clean product
        uint32_t e = ew ^ (uint32_t)fmask(f,1,it);
        uint32_t t = 0x80000000u - e;                              // 2 - e
        x = (uint32_t)((((uint64_t)x * t) >> 30) & M32) ^ (uint32_t)fmask(f,0,it+1); // Q2.30
        it++;
        o.x[it] = x;
        bool last = (it & 7) == n_iter, early = c.autostop && conv && !last;
        if(early || last){ o.conv = early; break; }
    }
    o.iters = it; o.cycles = 4 + 2*it;
    uint64_t P = (((uint64_t)ma * x) & ((1ULL<<56)-1)) ^ fmask(f,4,0);   // 56 bit
    o.fdet = !small_res((uint32_t)(((uint64_t)d * x) >> 31), 26);        // residual check in S_OUT
    bool ge1 = (P >> 54) & 1;
    uint32_t mant  = ge1 ? (uint32_t)((P>>31)&0xFFFFFF) : (uint32_t)((P>>30)&0xFFFFFF);
    bool guard     = ge1 ? (P>>30)&1 : (P>>29)&1;
    bool sticky    = ge1 ? (P & ((1ULL<<30)-1)) != 0 : (P & ((1ULL<<29)-1)) != 0;
    bool round_up  = guard && (sticky || (mant&1));
    uint32_t mr    = mant + round_up;
    bool carry     = (mr >> 24) & 1;
    int exp_f      = ea - eb + 127 - (ge1?0:1) + (carry?1:0);
    uint32_t frac  = carry ? 0 : (mr & 0x7FFFFF);
    if(exp_f >= 255)    { o.result = (sign<<31) | (0xFFu<<23); o.flags = 2; }
    else if(exp_f <= 0) { o.result = (sign<<31);               o.flags = 1; }
    else                  o.result = (sign<<31) | ((uint32_t)exp_f<<23) | frac;
    return o;
}

// ---- STAGE 4: square root path of fp_div.v (op = 1) ---------------------------
// y ~ 1/sqrt(x),  y(n+1) = y*(3 - x*y*y)/2,  sqrt = x*y
// x : Q1.31 in [0.25,1)  (m/4 if exponent E even, m/2 if E odd),  y : Q2.30 in (1,2)
// exact rounding: q = truncated root, round up iff (2q+1)^2 < 4*x*2^48
struct SqOut { uint32_t result; uint32_t y[9]; uint32_t flags; int iters; bool conv; int cycles; uint32_t plain; };

SqOut hw_sqrt(uint32_t a, int n_iter, const Cfg& c = Cfg()){
    SqOut o{}; o.cycles = 1;
    int ea = (a>>23)&0xFF; uint32_t fa = a & 0x7FFFFF, sa = a>>31;
    bool nan = ea==255 && fa, inf = ea==255 && !fa, zero = ea==0;   // zero or subnormal (DAZ)
    if(nan || zero || inf || sa){
        bool invalid = (nan && !(fa>>22&1)) || (sa && !zero && !nan);
        o.flags = invalid<<3;
        o.result = nan ? (a | 0x400000u) : zero ? (sa<<31) : sa ? 0x7FC00000u : 0x7F800000u;
        return o;
    }
    uint32_t ma = (1u<<23) | fa;
    uint32_t x  = (ea & 1) ? (ma << 6) : (ma << 7);                  // E even -> m/4, E odd -> m/2
    uint32_t y  = rsq_lut(((ea&1)<<3) | ((fa>>20)&7));
    o.y[0] = y;
    int it = 0;
    if(n_iter > 0) for(;;){
        uint32_t y2  = (uint32_t)((((uint64_t)y * y) >> 30) & M32);     // S_Y2  : y*y        mul_p[61:30]
        uint32_t xy2 = (uint32_t)((((uint64_t)x * y2) >> 31) & M32);    // S_XY2 : x*y*y      mul_p[62:31]
        bool conv = small_res(xy2, c.monk);
        uint32_t t = 0xC0000000u - xy2;                                  // 3 - x*y*y
        y = (uint32_t)((((uint64_t)y * t) >> 31) & M32);                // S_YT  : y*t/2      mul_p[62:31]
        it++;
        o.y[it] = y;
        bool last = (it & 7) == n_iter, early = c.autostop && conv && !last;
        if(early || last){ o.conv = early; break; }
    }
    o.iters = it; o.cycles = 4 + 3*it;
    uint64_t s = (uint64_t)x * y;                                    // S_SQ : Q3.61, root in [0.5,1)
    uint32_t q = (uint32_t)((s >> 37) & 0xFFFFFF);                    // truncated 24 bit root
    uint64_t chk = ((uint64_t)q << 1) | 1;                           // 2q+1
    uint32_t sq_hi = (uint32_t)(((chk*chk) >> 25) & 0x1FFFFFF);      // (2q+1)^2 [49:25]
    bool up = sq_hi < ((x >> 6) & 0x1FFFFFF);                        // sub32 borrow
    uint32_t mant = q + up;
    bool carry = (mant >> 24) & 1;
    uint32_t e = ((uint32_t)(ea + 127) >> 1) + carry;
    o.result = (e<<23) | (carry ? 0 : (mant & 0x7FFFFF));
    // plain round-half-up of the same root (what you get without the exact-rounding check)
    uint32_t mp = q + (uint32_t)((s>>36)&1); bool cp = (mp>>24)&1;
    o.plain = ((((uint32_t)(ea+127)>>1) + cp)<<23) | (cp ? 0 : (mp & 0x7FFFFF));
    return o;
}

static int ulp_err(uint32_t h, uint32_t r){
    if((h>>31)!=(r>>31)) return 1000000;
    uint32_t a=h&0x7FFFFFFF, b=r&0x7FFFFFFF; return (int)(a>b? a-b : b-a);
}

// IEEE reference for sqrt with the same DAZ / NaN conventions as the hardware
static uint32_t ref_sqrt(uint32_t a){
    int ea = (a>>23)&0xFF; uint32_t fa = a & 0x7FFFFF, sa = a>>31;
    if(ea==255 && fa) return a | 0x400000u;
    if(ea==0)         return sa<<31;
    if(sa)            return 0x7FC00000u;
    return f2u(sqrtf(u2f(a)));
}

// test vector list (unchanged from stage 3, first 4000 = vectors.hex)
static vector<pair<uint32_t,uint32_t>> make_vecs(){
    vector<pair<float,float>> directed = {
        {10,5},{14,3},{2,5},{10,3},{5,4},{4,-3},{-3,7},{-5,-13},
        {24605.84f,7},{0.125f,8},{23,12536.6f},{45,0.024f},
        {8,2},{1,1},{1,3},{3.14f,8.56f}
    };
    vector<pair<uint32_t,uint32_t>> vecs;
    for(auto&p:directed) vecs.push_back({f2u(p.first),f2u(p.second)});

    mt19937 rng(12345);
    auto rnd_normal = [&](){ // normal float, exponent kept moderate so quotient stays normal
        uint32_t s=rng()&1, e=64+rng()%127, f=rng()&0x7FFFFF; return (s<<31)|(e<<23)|f; };
    for(int i=0;i<5000;i++) vecs.push_back({rnd_normal(),rnd_normal()});
    // mantissa corner cases for the divisor
    uint32_t cornerF[] = {0x000000,0x7FFFFF,0x400000,0x100000,0x0FFFFF,0x700000,0x6FFFFF};
    for(uint32_t fb: cornerF) for(int i=0;i<20;i++){
        uint32_t a=rnd_normal(); uint32_t b=(0u<<31)|((100u)<<23)|fb; vecs.push_back({a,b}); }
    vecs.resize(4000);   // STAGE 4: same 4000 as the tb (VECTOR_COUNT), keeps vectors.hex unchanged
    return vecs;
}

// sqrt vector list: directed + random, 64 bit lines A|expected
static vector<uint32_t> make_sqrt_vecs(){
    vector<uint32_t> v = {
        0x3F800000,0x40000000,0x40800000,0x3E800000,0x3F000000,0x40400000,0x41100000,0x41800000, // 1 2 4 .25 .5 3 9 16
        0x40100000,0x3F7FFFFF,0x407FFFFF,0x3FFFFFFF,0x3F800001,0x7F7FFFFF,0x00800000,0x00800001, // near ties, max, min normal
        0x00000000,0x80000000,0x7F800000,0xFF800000,0xBF800000,0x80800000,0xC0000000,              // 0 -0 Inf -Inf -1 -min -2
        0x7FC00000,0x7FC12345,0xFFC00001,0x7F800001,0x7FA00000,                                   // NaNs (q, q payload, -q, s, s)
        0x00000001,0x007FFFFF,0x80000001,0x80400000                                               // subnormals (DAZ)
    };
    for(int k=-126;k<=127;k+=3) v.push_back((uint32_t)(k+127)<<23);  // 2^k, odd and even exponents
    mt19937 rng(777);
    while(v.size() < 1900){ uint32_t e = 1 + rng()%254; v.push_back((e<<23) | (rng() & 0x7FFFFF)); }
    while(v.size() < 2000) v.push_back(rng());                       // anything (negatives, specials)
    return v;
}

static FILE* fopen_lf(const char* n){ return fopen(n, "wb"); }   // LF line endings also on Windows

static const char* seed_name[3] = {"lut","line","pwl"};

// one per-vector dump in the tb results.csv format
static int dump(int sm, int iters, int autostop, int op, const char* fn){
    FILE* f = fopen_lf(fn);
    fprintf(f,"vec,a,b,result,expected,ulp,cycles,iters_used\n");
    Cfg c; c.seed = sm; c.autostop = autostop;
    if(op==0){
        auto vecs = make_vecs();
        for(size_t n=0;n<vecs.size();n++){ auto& v=vecs[n];
            uint32_t q = f2u(u2f(v.first)/u2f(v.second)); HwOut o = hw_div(v.first,v.second,iters,c);
            fprintf(f,"%zu,%08x,%08x,%08x,%08x,%d,%d,%d\n",n,v.first,v.second,o.result,q,ulp_err(o.result,q),o.cycles,o.iters); }
    } else {
        auto sv = make_sqrt_vecs();
        for(size_t n=0;n<sv.size();n++){ uint32_t q = ref_sqrt(sv[n]); SqOut o = hw_sqrt(sv[n],iters,c);
            fprintf(f,"%zu,%08x,%08x,%08x,%08x,%d,%d,%d\n",n,sv[n],0u,o.result,q,ulp_err(o.result,q),o.cycles,o.iters); }
    }
    fclose(f); return 0;
}

// fault campaign: golden = the hardware's own fault-free result (not IEEE)
struct FiCount { long masked=0, ulp1=0, sdc=0, due=0, det=0, sdc_det=0; };
static FiCount fi_point(const vector<pair<uint32_t,uint32_t>>& vecs, size_t nv, int sel, int k, int bit){
    FiCount r; Cfg c; c.fi.en = true; c.fi.sel = sel; c.fi.iter = k; c.fi.bit = bit;
    for(size_t n=0;n<nv;n++){
        uint32_t g = hw_div(vecs[n].first, vecs[n].second, 3).result;
        HwOut o = hw_div(vecs[n].first, vecs[n].second, 3, c);
        int e = ulp_err(o.result, g);
        if(e==0) r.masked++; else if(e==1) r.ulp1++; else { r.sdc++; if(o.fdet) r.sdc_det++; }
        if(o.fdet) r.det++;
    }
    return r;
}
static const char* site_name[5] = {"x","e","d","mant_A","prod"};

// same (site, iter, bit) list as tb_fault.v
static void fault_subset(const char* fn){
    auto vecs = make_vecs();
    FILE* f = fopen_lf(fn);
    fprintf(f,"site,iter,bit,n,masked,ulp1,sdc,due,detected,sdc_detected\n");
    int bits32[] = {0,5,10,15,20,25,28,31}, bitsA[] = {0,5,10,15,20,23}, bitsP[] = {0,10,20,30,40,50,54,55};
    int nk[5] = {4,3,1,1,1};
    for(int s=0;s<5;s++) for(int k=0;k<nk[s];k++){
        int* bl = s==3 ? bitsA : s==4 ? bitsP : bits32; int nb = s==3 ? 6 : 8;
        for(int j=0;j<nb;j++){ FiCount r = fi_point(vecs,200,s,k,bl[j]);
            fprintf(f,"%s,%d,%d,%d,%ld,%ld,%ld,%ld,%ld,%ld\n",site_name[s],k,bl[j],200,r.masked,r.ulp1,r.sdc,r.due,r.det,r.sdc_det); }
    }
    fclose(f);
}

int main(int argc, char** argv){
    if(argc==7 && string(argv[1])=="--dump") return dump(atoi(argv[2]),atoi(argv[3]),atoi(argv[4]),atoi(argv[5]),argv[6]);
    if(argc==3 && string(argv[1])=="--fault-subset"){ fault_subset(argv[2]); return 0; }

    auto vecs = make_vecs();

    // ---------------- vectors.hex (A B expected) ----------------
    FILE* fv=fopen_lf("vectors.hex");
    for(size_t i=0;i<vecs.size();i++){ auto&v=vecs[i]; float q=u2f(v.first)/u2f(v.second);   // no newline after the last line (as in git)
        fprintf(fv,"%08x%08x%08x%s",v.first,v.second,f2u(q),i+1<vecs.size()?"\n":""); }
    fclose(fv);

    // ---------------- STAGE 4: constant tables for the RTL (paste into seed_linear.v / rsqrt_lut.v) ----------------
    printf("seed_linear.v constants (c0 Q2.30, c1 Q2.14):\n");
    for(int pwl=0;pwl<2;pwl++) for(int i=0;i<(pwl?8:1);i++){ uint32_t c0,c1; lin_coef(pwl,i,c0,c1);
        printf("  %s[%d]: c0 = 32'h%08X (%.6f)  c1 = 16'h%04X (%.6f)\n",pwl?"pwl ":"line",i,c0,c0/1073741824.0,c1,c1/16384.0); }
    printf("rsqrt_lut.v entries (Q2.30):\n");
    for(int i=0;i<16;i++) printf("  4'd%-2d: y0 = 32'h%08X  (%.6f)\n",i,rsq_lut(i),rsq_lut(i)/1073741824.0);

    // ---------------- accuracy vs iteration (4000 vectors = vectors.hex, 1M random sqrt) ----------------
    mt19937 rs(2024);
    vector<uint32_t> sq_rand(1000000);
    for(auto& a: sq_rand){ uint32_t e = 1 + rs()%254; a = (e<<23) | (rs() & 0x7FFFFF); }
    FILE* fa=fopen_lf("accuracy_vs_iter.csv");
    fprintf(fa,"iterations,latency_cycles");
    for(int m=0;m<3;m++) fprintf(fa,",%s_exact_pct,%s_within1ulp_pct,%s_max_ulp",seed_name[m],seed_name[m],seed_name[m]);
    fprintf(fa,",sqrt_latency_cycles,sqrt_exact_pct,sqrt_within1ulp_pct,sqrt_max_ulp\n");
    printf("\nAccuracy (divide: the 4000 vectors of vectors.hex, sqrt: 1M random positive normals)\n");
    printf("%-6s %-5s | %-22s | %-22s | %-22s | %-5s %-22s\n","iters","lat","lut  exact/<=1/max","line exact/<=1/max","pwl  exact/<=1/max","lat","sqrt exact/<=1/max");
    for(int it=0; it<=4; it++){
        int lat = 4 + 2*it; // IDLE capture + 2/iter + S_Q + S_OUT + done
        fprintf(fa,"%d,%d",it,lat); printf("%-6d %-5d",it,lat);
        for(int m=0;m<3;m++){
            Cfg c; c.seed=m; long ex=0,one=0; int mx=0;
            for(auto&v:vecs){ float q=u2f(v.first)/u2f(v.second);
                int e=ulp_err(hw_div(v.first,v.second,it,c).result,f2u(q));
                if(e==0)ex++; else if(e==1)one++; if(e>mx)mx=e; }
            double n=vecs.size();
            fprintf(fa,",%.4f,%.4f,%d",100*ex/n,100*(ex+one)/n,mx);
            printf(" | %7.3f %7.3f %6d",100*ex/n,100*(ex+one)/n,mx);
        }
        long ex=0,one=0; int mx=0;
        for(uint32_t a: sq_rand){ int e=ulp_err(hw_sqrt(a,it).result,ref_sqrt(a)); if(e==0)ex++; else if(e==1)one++; if(e>mx)mx=e; }
        double n=sq_rand.size();
        fprintf(fa,",%d,%.4f,%.4f,%d\n",4+3*it,100*ex/n,100*(ex+one)/n,mx);
        printf(" | %-5d %7.3f %7.3f %6d\n",4+3*it,100*ex/n,100*(ex+one)/n,mx);
    }
    fclose(fa);

    // ---------------- convergence: worst relative error per iteration, exhaustive over the mantissa ----------------
    FILE* fc=fopen_lf("convergence.csv");
    fprintf(fc,"mode,iter,rel_err,correct_bits\n");
    printf("\nWorst-case relative error of x_k (divide: all 2^23 mantissas, sqrt: all 2^24 parity x mantissa):\n");
    for(int m=0;m<3;m++){
        double worst[6]={0}; Cfg c; c.seed=m;
        for(uint32_t fb=0; fb<(1u<<23); fb++){
            uint32_t b=(127u<<23)|fb; HwOut o=hw_div(f2u(1.0f),b,4,c);
            double d=(double)((1u<<23)|fb)/(1<<24);
            for(int k=0;k<=4;k++){ double re=fabs(o.x[k]/(double)(1<<30)*d-1.0); if(re>worst[k]) worst[k]=re; }
        }
        printf("  %-5s:",seed_name[m]);
        for(int k=0;k<=4;k++){ printf("  x%d %.2f bits",k,-log2(worst[k]));
            fprintf(fc,"%s,%d,%.6e,%.2f\n",seed_name[m],k,worst[k],-log2(worst[k])); }
        printf("\n");
    }
    {
        double worst[6]={0}; long ex=0, plain_ex=0, plain_1=0; int mx=0;
        for(uint32_t p=0;p<2;p++) for(uint32_t fa2=0; fa2<(1u<<23); fa2++){
            uint32_t a = ((127u+p)<<23) | fa2;                       // ea = 127 (E even) or 128 (E odd)
            SqOut o = hw_sqrt(a,4);
            uint32_t ma=(1u<<23)|fa2; double x = ((127+p)&1) ? ma/(double)(1u<<25) : ma/(double)(1u<<24);
            for(int k=0;k<=4;k++){ double re=fabs(o.y[k]/(double)(1<<30)*sqrt(x)-1.0); if(re>worst[k]) worst[k]=re; }
            SqOut o3 = hw_sqrt(a,3); uint32_t r = ref_sqrt(a);
            int e = ulp_err(o3.result,r); if(e==0) ex++; if(e>mx) mx=e;
            int ep = ulp_err(o3.plain,r); if(ep==0) plain_ex++; else if(ep==1) plain_1++;
        }
        printf("  sqrt :"); for(int k=0;k<=4;k++){ printf("  y%d %.2f bits",k,-log2(worst[k]));
            fprintf(fc,"sqrt,%d,%.6e,%.2f\n",k,worst[k],-log2(worst[k])); }
        printf("\n\nExhaustive sqrt check, 3 iterations, all 2^24 (mantissa x exponent parity):\n");
        printf("  exact rounding    : %ld / %u correctly rounded, max ULP %d\n",ex,1u<<24,mx);
        printf("  plain (no check)  : %.4f %% exact, %.4f %% <=1 ULP\n",100.0*plain_ex/(1u<<24),100.0*(plain_ex+plain_1)/(1u<<24));
    }
    fclose(fc);
    {   // sqrt exponent: every normal exponent with mantissa 1.0 and 1.5
        int bad=0;
        for(uint32_t e=1;e<=254;e++) for(uint32_t fr: {0u,0x400000u}){ uint32_t a=(e<<23)|fr;
            if(hw_sqrt(a,3).result!=ref_sqrt(a)) bad++; }
        printf("  exponent sweep    : %d mismatches over 508 inputs\n",bad);
    }

    // ---------------- convergence monitor: iteration histogram and accuracy vs threshold ----------------
    mt19937 rm(4242);
    vector<pair<uint32_t,uint32_t>> mv(200000);
    auto rnd_normal = [&](){ uint32_t s=rm()&1, e=64+rm()%127, f=rm()&0x7FFFFF; return (s<<31)|(e<<23)|f; };
    for(auto& v: mv) do { v.first = rnd_normal(); v.second = rnd_normal(); }   // keep only normal quotients (no FTZ cases)
        while(((f2u(u2f(v.first)/u2f(v.second))>>23)&0xFF) == 0);
    FILE* fm=fopen_lf("monitor.csv");
    fprintf(fm,"mode,threshold_bits,cap,avg_iters,avg_cycles,exact_pct,within1ulp_pct,max_ulp,h0,h1,h2,h3,h4\n");
    printf("\nConvergence monitor (200k random, cap = 3 iterations):\n");
    printf("  %-5s %-4s %-9s %-10s %-9s %-9s %-6s  histogram 0..4\n","mode","T","avg_it","avg_cyc","exact%","<=1ULP%","maxULP");
    for(int m=0;m<4;m++) for(int k=13;k<=17;k++){
        Cfg c; c.seed = m<3 ? m : 0; c.autostop = true; c.monk = k;
        long ex=0,one=0,sit=0,scy=0,h[5]={0}; int mx=0;
        for(auto& v: mv){
            uint32_t res, ref; int its, cyc;
            if(m<3){ HwOut o=hw_div(v.first,v.second,3,c); res=o.result; its=o.iters; cyc=o.cycles; ref=f2u(u2f(v.first)/u2f(v.second)); }
            else   { uint32_t a=v.first&0x7FFFFFFF; SqOut o=hw_sqrt(a,3,c); res=o.result; its=o.iters; cyc=o.cycles; ref=ref_sqrt(a); }
            int e=ulp_err(res,ref); if(e==0)ex++; else if(e==1)one++; if(e>mx)mx=e;
            sit+=its; scy+=cyc; h[its]++;
        }
        double n=mv.size(); const char* nm = m<3 ? seed_name[m] : "sqrt";
        fprintf(fm,"%s,%d,3,%.4f,%.4f,%.4f,%.4f,%d,%ld,%ld,%ld,%ld,%ld\n",nm,k,sit/n,scy/n,100*ex/n,100*(ex+one)/n,mx,h[0],h[1],h[2],h[3],h[4]);
        printf("  %-5s 2^-%-2d %-9.4f %-10.4f %-9.4f %-9.4f %-6d  %ld %ld %ld %ld %ld\n",nm,k,sit/n,scy/n,100*ex/n,100*(ex+one)/n,mx,h[0],h[1],h[2],h[3],h[4]);
    }
    fclose(fm);

    // ---------------- STAGE 4: vectors_sqrt.hex (A expected) ----------------
    auto sv = make_sqrt_vecs();
    FILE* fs=fopen_lf("vectors_sqrt.hex");
    long sex=0; int smx=0;
    for(uint32_t a: sv){ uint32_t r=ref_sqrt(a); fprintf(fs,"%08x%08x\n",a,r);
        int e=ulp_err(hw_sqrt(a,3).result,r); if(e==0) sex++; if(e>smx) smx=e; }
    fclose(fs);
    printf("\nvectors_sqrt.hex: %zu vectors, hw at 3 iterations: %ld exact, max ULP %d\n",sv.size(),sex,smx);

    // ---------------- STAGE 4: fault campaign (2000 vectors per point, LUT seed, 3 iterations) ----------------
    {
        long fa_cnt=0; double worst=0;
        for(size_t n=0;n<vecs.size();n++){ HwOut o=hw_div(vecs[n].first,vecs[n].second,3); if(o.fdet) fa_cnt++;
            uint32_t mb=(1u<<23)|(vecs[n].second&0x7FFFFF); double d=(mb<<7)/2147483648.0;
            double re=fabs(o.x[3]/1073741824.0*d-1); if(re>worst) worst=re; }
        printf("\nResidual check, fault free: %ld false alarms in %zu vectors, worst |1-d*x3| = %.3e\n",fa_cnt,vecs.size(),worst);
    }
    FILE* ff=fopen_lf("fault_campaign.csv");
    fprintf(ff,"site,iter,bit,masked_pct,ulp1_pct,sdc_pct,due_pct,detected_pct,sdc_detected_pct\n");
    int nk[5] = {4,3,1,1,1}, nbits[5] = {32,32,32,24,56};
    printf("\nFault campaign (2000 vectors per point): bits 0..N never give an SDC (result stays within 1 ULP)\n");
    long tot_sdc[5]={0}, tot_sdc_det[5]={0};
    for(int s=0;s<5;s++) for(int k=0;k<nk[s];k++){
        int hi_ok=-1; bool run=true;
        for(int bit=0;bit<nbits[s];bit++){
            FiCount r = fi_point(vecs,2000,s,k,bit); double n=2000;
            fprintf(ff,"%s,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",site_name[s],k,bit,100*r.masked/n,100*r.ulp1/n,100*r.sdc/n,100*r.due/n,
                    100*r.det/n, r.sdc? 100.0*r.sdc_det/r.sdc : 100.0);
            if(run && r.sdc==0) hi_ok=bit; else run=false;
            tot_sdc[s]+=r.sdc; tot_sdc_det[s]+=r.sdc_det;
        }
        printf("  %-6s k=%d : bits 0..%d corrected\n",site_name[s],k,hi_ok);
    }
    for(int s=0;s<5;s++) printf("  %-6s : %ld SDC runs, %.2f %% of them detected by the residual check\n",
                                site_name[s],tot_sdc[s],tot_sdc[s]? 100.0*tot_sdc_det[s]/tot_sdc[s] : 0.0);
    fclose(ff);

    // ---------------- print the special cases ----------------
    uint32_t spec[][2] = {
        {0x00000000,0x00000000},{0x3F800000,0x00000000},{0x7F800000,0xFF800000},{0x7F800000,0x40000000},
        {0x3F800000,0x7F800000},{0x7FC12345,0x3F800000},{0x7F800001,0x3F800000},{0x00000001,0x3F800000},
        {0x3F800000,0x00400000},{0x7149F2CA,0x0DA24260},{0x0DA24260,0x7149F2CA}};
    printf("\nSpecial cases (flags = invalid,div_by_zero,overflow,underflow):\n");
    for(auto&v:spec){ HwOut o=hw_div(v[0],v[1],3);
        printf("  %08x / %08x  hw=%08x  flags=%d%d%d%d\n",v[0],v[1],o.result,
               o.flags>>3&1,o.flags>>2&1,o.flags>>1&1,o.flags&1); }
    printf("\nWrote %zu vectors to vectors.hex, %zu to vectors_sqrt.hex\n",vecs.size(),sv.size());
}
