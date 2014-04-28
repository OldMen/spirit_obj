#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <functional>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/operators.hpp>
#include <boost/spirit/include/support_multi_pass.hpp>
#include <boost/spirit/include/classic_position_iterator.hpp>

using namespace std;
using namespace boost::spirit;

struct Vertex
{
	explicit Vertex() : x( 0. ), y( 0. ), z( 0. ), w( 1. ) {}
	explicit Vertex( double vx, double vy, double vz, double vw = 1 ) : x( vx ), y( vy ), z( vz ), w( vw ) {}

	double	x;
	double	y;
	double	z;
	double	w;
};
BOOST_FUSION_ADAPT_STRUCT(
	Vertex,
	(double, x)
	(double, y)
	(double, z)
	(double, w)
)

struct Normal
{
	explicit Normal() : x( 0. ), y( 0. ), z( 0. ) {}
	explicit Normal( double vx, double vy, double vz ) : x( vx ), y( vy ), z( vz ) {}

	double	x;
	double	y;
	double	z;
};
BOOST_FUSION_ADAPT_STRUCT(
	Normal,
	(double, x)
	(double, y)
	(double, z)
)

struct TexCoord
{
	explicit TexCoord() : u( 0. ), v( 0. ), w( 0. ) {}
	explicit TexCoord( double vu, double vv, double vw = 0 ) : u( vu ), v( vv ), w( vw ) {}

	double	u;
	double	v;
	double	w;
};
BOOST_FUSION_ADAPT_STRUCT(
	TexCoord,
	(double, u)
	(double, v)
	(double, w)
)

struct Index
{
	explicit Index() : v( 0 ), vn( 0 ), vt( 0 ) {}
	explicit Index( int vv, int vvn, int vvt ) : v( vv ), vn( vvn ), vt( vvt ) {}

	int		v;
	int		vn;
	int		vt;
};
BOOST_FUSION_ADAPT_STRUCT(
	Index,
	(int, v)
	(int, vn)
	(int, vt)
)

typedef deque<Vertex>	VertexList;
typedef deque<Normal>	NormalList;
typedef deque<TexCoord>	TexCoordList;
typedef deque<Index>	Face;
typedef deque<Face>		FaceList;

struct ModelData
{
	void	dump() const
	{
		cout << endl << "vertex list:" << endl;
		for( auto i : v )
			cout << i.x << ", " << i.y << ", " << i.z << ", " << i.w << endl;

		cout << endl << "normal list:" << endl;
		for( auto i : vn )
			cout << i.x << ", " << i.y << ", " << i.z << endl;

		cout << endl << "texcoord list:" << endl;
		for( auto i : vt )
			cout << i.u << ", " << i.v << ", " << i.w << endl;

		cout << endl << "face list:" << endl;
		for( auto i : f )
		{
			for( auto y : i )
			{
				cout << y.v << "/" << y.vn << "/" << y.vt << ' ';
			}

			cout << endl;
		}
	}

	VertexList		v;
	NormalList		vn;
	TexCoordList	vt;
	FaceList		f;
};
BOOST_FUSION_ADAPT_STRUCT(
	ModelData,
	(VertexList,	v)
	(NormalList,	vn)
	(TexCoordList,	vt)
	(FaceList,		f)
)

// ================================================================================================

template <class ForwardIterator, class Skipper>
class ObjParser : public qi::grammar<ForwardIterator, Skipper, ModelData()>
{
public:
	ObjParser() : ObjParser::base_type( m_start )
	{
		using boost::phoenix::at_c;

		m_start = *(
					 m_vertex_lst[ at_c<0>(_val) = _1 ] |
					 m_normal_lst[ at_c<1>(_val) = _1 ] |
					 m_texcoord_lst[ at_c<2>(_val) = _1 ] |
					 m_face_lst[ at_c<3>(_val) = _1 ]
				   ) > qi::eoi;

		m_vertex_lst = m_vertex > *m_vertex;
		m_vertex = lit( "v" ) >> double_[ at_c<0>(_val) = _1 ] > double_[ at_c<1>(_val) = _1 ] > double_[ at_c<2>(_val) = _1 ] > -double_[ at_c<3>(_val) = _1 ];

		m_normal_lst = m_normal > *m_normal;
		m_normal = lit( "vn" ) > double_[ at_c<0>(_val) = _1 ] > double_[ at_c<1>(_val) = _1 ] > double_[ at_c<2>(_val) = _1 ];

		m_texcoord_lst = m_texcoord > *m_texcoord;
		m_texcoord = lit( "vt" ) > double_[ at_c<0>(_val) = _1 ] > double_[ at_c<1>(_val) = _1 ] > -double_[ at_c<2>(_val) = _1 ];

		m_face_lst = m_face > *m_face;
		m_face = lit( "f" ) > m_index > m_index > m_index > *m_index;
		m_index = int_[ at_c<0>(_val) = _1 ] > -( '/' > -int_[ at_c<1>(_val) = _1 ] > -( '/' > int_[ at_c<2>(_val) = _1 ] ) );
	}

private:
	qi::rule<ForwardIterator, Skipper, ModelData()>		m_start;
	qi::rule<ForwardIterator, Skipper, VertexList()>	m_vertex_lst;
	qi::rule<ForwardIterator, Skipper, Vertex()>		m_vertex;
	qi::rule<ForwardIterator, Skipper, NormalList()>	m_normal_lst;
	qi::rule<ForwardIterator, Skipper, Normal()>		m_normal;
	qi::rule<ForwardIterator, Skipper, TexCoordList()>	m_texcoord_lst;
	qi::rule<ForwardIterator, Skipper, TexCoord()>		m_texcoord;
	qi::rule<ForwardIterator, Skipper, FaceList()>		m_face_lst;
	qi::rule<ForwardIterator, Skipper, Face()>			m_face;
	qi::rule<ForwardIterator, Skipper, Index()>			m_index;
};

// ================================================================================================

template<class ForwardIterator, class Skipper>
inline bool parse( ForwardIterator first, ForwardIterator last, Skipper &s, ModelData &r )
{
	ObjParser<ForwardIterator, Skipper> p;
	return qi::phrase_parse( first, last, p, s, r );
}

template<class Skipper>
inline bool parse( istream &is, const string &filename, Skipper &s, ModelData &r )
{
	typedef istreambuf_iterator<char> BaseIterator;
	typedef multi_pass<BaseIterator> ForwardIterator;
	typedef classic::position_iterator2<ForwardIterator> PosIterator;

	try
	{
		BaseIterator it( is );

		ForwardIterator fwd_beg = make_default_multi_pass( it );
		ForwardIterator fwd_end;

		PosIterator beg( fwd_beg, fwd_end, filename );
		PosIterator end;

		ObjParser<PosIterator, Skipper> p;
		return qi::phrase_parse( beg, end, p, s, r );
	}
	catch( const qi::expectation_failure<PosIterator> &e )
	{
		const classic::file_position_base<string> &pos = e.first.get_position();
		cout << "Parse error at file '" << pos.file << "' line " << pos.line << " column " << pos.column << endl
			 <<	"'" << e.first.get_currentline() << "'" << endl
			 <<	setw( pos.column ) << " " << "^- here" << endl;
		return false;
	}
}

// ================================================================================================

int main()
{
	string str =
			"v 10 10 10 10 # dfdfdfsdfsdf\n"
			"v 20 20 20\n"
			"v 30 30 30 30 # dfdsfsdfsdfsdfsdfsdfsdf # sddsfdsf\n"
			"v 40 40 40\n"
			"# comment\n"
			"vn 220 220 220\n"
			"vn 220 220 220\n"
			"# comment\n"
			"vt 20 20 20\n"
			"vt 20 20\n"
			"vt 200 200 200\n"
			"f 1 2 3\n"
			"f 1/1 2/2 3/3 \\\n"
			"4/4 5/5 6/6\n"
			"f 1/1/1 2/2/2 3/3/3\n"
			"f 1//1 2//2 3//3\n";


	istringstream is( str );
	noskipws( is );

	ModelData result;
	auto sp = qi::space | '\\' | '#' >> *(qi::char_ - qi::eol) >> qi::eol;
	cout << "Result: " << parse( is, "local", sp, result ) << endl;
	result.dump();

	return 0;
}
