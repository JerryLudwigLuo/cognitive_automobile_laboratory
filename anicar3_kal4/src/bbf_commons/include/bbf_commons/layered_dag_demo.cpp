#include "LayeredDAGSolver.hpp"
#include "open_png.hpp"

typedef int64_t COST_TYPE;

const int NUM_LAYERS = 200;
const int LAYER_DIM1 = 128;
const int LAYER_DIM2 = 20;
const int DELTA_DIM1 = 1;
const int DELTA_DIM2 = 1;

typedef LayeredDAGSolver<COST_TYPE, NUM_LAYERS, LAYER_DIM1, LAYER_DIM2, DELTA_DIM1, DELTA_DIM2> Solver;


const int BASE_WIDTH = 12;

std::vector<COST_TYPE> compute_data_cost( const std::vector<unsigned char>& image_data )
{
    std::vector<COST_TYPE> data_cost( Solver::num_nodes(), 0 );
    auto data_it = data_cost.begin();

    for( auto row_it = image_data.begin(); row_it < image_data.end(); row_it += Solver::dim1() )
    {
        const auto col_end = row_it + Solver::dim1();
        for( auto col_it = row_it; col_it < col_end; col_it++ )
        {
            auto width_end = col_it + Solver::dim2();
            for( auto width_it = col_it + BASE_WIDTH; width_it != width_end; width_it++ )
            {
                const double w_map = 1.0; // blackness in the map is good

                if( width_it < col_end )
                    *data_it = w_map * (*col_it) * (*width_it); // brightness left border times brightness right border
                else
                    *data_it = std::numeric_limits<COST_TYPE>::max()/8;

                // safety margin
                if( row_it - image_data.begin() <= Solver::max_delta_dim2() * Solver::dim1() )
                    *data_it = std::numeric_limits<COST_TYPE>::max()/8;

                if( row_it - image_data.begin() > ( Solver::dim2() - Solver::max_delta_dim2() ) * Solver::dim1() )
                    *data_it = std::numeric_limits<COST_TYPE>::max()/8;

                data_it++;
            }
        }
    }

    return data_cost;
}

std::vector<COST_TYPE> compute_edge_cost()
{
    std::vector<COST_TYPE> edge_costs;

    for( int i=-Solver::max_delta_dim1(); i<Solver::max_delta_dim1()+1; i++ )
    {
        for( int j=-Solver::max_delta_dim1(); j<Solver::max_delta_dim2()+1; j++ )
        {
            const double w_straightness = 1; // straightness is good.
            const double w_evenness     = 1; // constant width of corridor is good, too.

            edge_costs.push_back( w_straightness*i*i + w_evenness*j*j );
        }
    }

    return edge_costs;
}

int main()
{
    std::vector<unsigned char> image_data;
    int w, h, line_step;
    MRT::open_png<boost::gil::gray8_pixel_t>( "demo.png", image_data, w, h, line_step );

    assert( line_step == w );
    assert( w == Solver::dim1() );
    assert( h == Solver::num_layers() );

    std::vector<COST_TYPE> data_cost = compute_data_cost( image_data );
    std::vector<COST_TYPE> edge_cost = compute_edge_cost();

//    for( auto v : data_cost )
//        std::cout << v << "\n";

    Solver solver( data_cost, edge_cost );
    solver.expand();
    solver.unroll();
}
