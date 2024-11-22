#include "ampl_test.h"

#include <ampl/ampl.h>

#include "common.h"
#include "filebuffer.h"
#include "random.h"
#include "vector.h"

#define TEST_FOLDER_WITHOUT_SEPARATOR "ampl_test"
#define TEST_FOLDER "ampl_test/"

class AMPLErrorHandler : public ampl::ErrorHandler
{
public:
    void error(const ampl::AMPLException& ex) override
    {
        const std::string& source_name = ex.getSourceName();
        int line_number = ex.getLineNumber();
        int get_offset = ex.getOffset();
        const std::string& message = ex.getMessage();

        printf("ErrHandler: %s (%s/%d/%d)", message.c_str(), source_name.c_str(), line_number, get_offset);
    }

    void warning(const ampl::AMPLException& ex) override
    {
        const std::string& source_name = ex.getSourceName();
        int line_number = ex.getLineNumber();
        int get_offset = ex.getOffset();
        const std::string& message = ex.getMessage();

        printf("ErrHandler: %s (%s/%d/%d)", message.c_str(), source_name.c_str(), line_number, get_offset);
    }
};

static ampl::Environment* s_env;
static ampl::AMPL* s_ampl;
static AMPLErrorHandler s_error_handler;

static void test_two_variable_linear();
static void test_two_variable_linear_in_ampl();
static void test_data_functions();
static void test_data_vector_declarations();
static void test_simple_linear_least_squares();
static void test_global_lls();

typedef void (*TestFunc)();
static TestFunc s_tests[] =
{
    test_two_variable_linear,
    test_two_variable_linear_in_ampl,
    test_data_functions,
    test_data_vector_declarations,
    test_simple_linear_least_squares,
    test_global_lls,
};

void ampl_test(char* argv[])
{
    if (directory_is_exist(TEST_FOLDER_WITHOUT_SEPARATOR) == false)
    {
        directory_create(TEST_FOLDER_WITHOUT_SEPARATOR);
    }

    s_env = new ampl::Environment(argv[1]);
    s_ampl = new ampl::AMPL(*s_env);
    s_ampl->setErrorHandler(&s_error_handler);
    
    printf("%s\n", s_ampl->getOption("version").value().c_str());
    printf("Solver : %s\n", s_ampl->getOption("solver").value().c_str());

    s_ampl->setOption("solver", "Gurobi");
    printf("Solver : %s\n", s_ampl->getOption("solver").value().c_str());
    

    int test_count = sizeof(s_tests) / sizeof(s_tests[0]);

    for (int i = 0; i < test_count; ++i)
    {
        s_tests[i]();
    }

    delete s_ampl;
    delete s_env;
}

static void save_data(const char* path, const char* data)
{
    FileBuffer fb; 
    filebuffer_open(&fb, path, "wb");

    filebuffer_write_str(&fb, data);

    filebuffer_close(&fb);
}

static void test_two_variable_linear()
{
    s_ampl->reset();

    /*
    * Refer to Chapter 1.1/1.2 A two-variable linear program
    * 
    * Tons per hour:    Bands 200
    *                   Coils 140
    * Profit per ton:   Bands $25
    *                   Coils $30
    * Maximum tons:     Bands 6,000
    *                   Coils 4,000
    * 40 hours of production time are available.
    * 
    * Decision variables.
    *   X_B for the number of tons of bands to be produced.
    *   X_C for tons of coils.
    * 
    * The total hours to produce all these tons:
    *   (hours to make a ton of bands) x X_B + (hours to make a ton of coils) x X_C
    * This number cannot exceed the 40 hours available. We have a constraint on the variables:
    *   (1/200) X_B + (1/140)X_C <= 40.
    * Production limits:
    *   0 <= X_B <= 6000
    *   0 <= X_C <= 4000
    * Total Profit:
    *   (profit per ton of bands) x X_B + (profit per ton of coils) x X_C
    * Linear Program:
    *   Maximize    25X_B + 30X_C
    *   Subject to  (1/200) X_B + (1/140) X_C <= 40
    *               0 <= X_B <= 6000
    *               0 <= X_C <= 4000
    */

    const char* model =
        "var XB;\n"
        "var XC;\n"
        "maximize Profit: 25 * XB + 30 * XC;\n"
        "subject to Time: (1/200) * XB + (1/140) * XC <= 40;\n"
        "subject to B_limit: 0 <= XB <= 6000;\n"
        "subject to C_limit: 0 <= XC <= 4000;\n";

    save_data(TEST_FOLDER"test_two_variable.mod", model);

    s_ampl->read(TEST_FOLDER"test_two_variable.mod");

    s_ampl->solve();

    ampl::Objective objective = s_ampl->getObjective("Profit");
    ampl::Variable xb = s_ampl->getVariable("XB");
    ampl::Variable xc = s_ampl->getVariable("XC");

    printf("Profit %f with %f tons of bands and %f tons of coils", objective.value(), xb.value(), xc.value());
}

static void test_two_variable_linear_in_ampl()
{
    s_ampl->reset();

    const char* model =
        // Given:
        "set P;\n"
        "param a {j in P};\n"   // tons per hour of product j, for each j in P.
        "param b;\n"            // hours available at the mill.
        "param c {j in P};\n"   // profit per ton of product j, for each j in P.
        "param u {j in P};\n"   // maximum tons of product j, for each j in P.
        // Define variables:
        "var X {j in P};\n"     // tons of product j to be made,  for each j in P.
        // Maximize:
        "maximize Total_Profit: sum {j in P} c[j] * X[j];\n"
        // Subject to:
        "subject to Time: sum {j in P} (1 / a[j]) * X[j] <= b;\n"
        "subject to Limit {j in P}: 0 <= X[j] <= u[j];\n";

    const char* data =
        "set P := bands coils;\n"
        "param:    a    c    u :=\n"
        "  bands  200  25  6000\n"
        "  coils  140  30  4000 ;\n"
        "param b := 40;\n";

    save_data(TEST_FOLDER"test_two_variable_in_ampl.mod", model);
    save_data(TEST_FOLDER"test_two_variable_in_ampl.dat", data);

    s_ampl->read(TEST_FOLDER"test_two_variable_in_ampl.mod");
    s_ampl->readData(TEST_FOLDER"test_two_variable_in_ampl.dat");
    
    s_ampl->solve();

    ampl::Objective total_profit = s_ampl->getObjective("Total_Profit");

    ampl::Variable x = s_ampl->getVariable("X");
    ampl::DataFrame xvs = x.getValues();
    std::string s = xvs.toString();
    printf("Num Indices : %zu / Num Rows : %zu / Num Cols : %zu\nData :\n %s\n", 
        xvs.getNumIndices(), xvs.getNumRows(), xvs.getNumCols(), s.c_str());

    ampl::DataFrame::Row row0 = xvs.getRowByIndex(0);
    ampl::DataFrame::Row row1 = xvs.getRowByIndex(1);

    printf("%s %f\n", row0[0].c_str(), row0[1].dbl());
    printf("%s %f\n", row1[0].c_str(), row1[1].dbl());
}

static void test_data_functions()
{
    s_ampl->reset();
    
    const char* model =
        "set FOOD;\n"
        "param cost {FOOD};\n"
        "param f_min {FOOD};\n"
        "param f_max {FOOD};\n"

        "set NUTR;\n"
        "param n_min {NUTR} >= 0;\n"
        "param n_max {i in NUTR} >= n_min[i];\n"

        "param amt {NUTR, FOOD} >= 0;\n"
        ;

    save_data(TEST_FOLDER"test_data_functions.mod", model);

    s_ampl->read(TEST_FOLDER"test_data_functions.mod");

    {
        const char* foods[] = { "BEEF", "CHK", "FISH", "HAM",
                           "MCH",  "MTL", "SPG",  "TUR" };
        double costs[] = { 3.59, 2.59, 2.29, 2.89, 1.89, 1.99, 1.99, 2.49 };
        double fmin[] = { 2, 2, 2, 2, 2, 2, 2, 2 };
        double fmax[] = { 10, 10, 10, 10, 10, 10, 10, 10 };

        ampl::DataFrame df(1, "FOOD");
        df.setColumn("FOOD", foods, 8);
        df.addColumn("cost", costs);
        df.addColumn("f_min", fmin);
        df.addColumn("f_max", fmax);

        std::string s = df.toString();
        printf("%s\n", s.c_str());

        /*
        * In order to call the s_ampl->setData(df, some_variable_name),
        * you have to specify set some_variable_name;
        */
        s_ampl->setData(df, "FOOD");

        df = s_ampl->getData("{i in FOOD} cost[i] / 2.0");
        s = df.toString();
        printf("%s\n", s.c_str());
    }

    {
        /*
        * you can change nmin or nmax to cause an error 
        * that does not satisfy the condition in param declarations of the model.
        * The error is checked in the getData 
        */
        const char* nutrients[] = { "A", "C", "B1", "B2", "NA", "CAL" };
        double nmin[] = { 700, 700, 700, 700, 0, 16000 };
        double nmax[] = { 20000, 20000, 20000, 20000, 50000, 24000 };
        ampl::DataFrame df(1, "NUTR");
        df.setColumn("NUTR", nutrients, 6);
        df.addColumn("n_min", nmin);
        df.addColumn("n_max", nmax);
        s_ampl->setData(df, "NUTR");

        std::string s = df.toString();
        printf("%s\n", s.c_str());

        df = s_ampl->getData("{i in {NUTR}} n_max[i] / 2.0");
        s = df.toString();
        printf("%s\n", s.c_str());
    }

    {
        const char* nutrients[] = { "A", "C", "B1", "B2", "NA", "CAL" };
        const char* foods[] = { "BEEF", "CHK", "FISH", "HAM",
                           "MCH",  "MTL", "SPG",  "TUR" };
        double amounts[6][8] = { {60, 8, 8, 40, 15, 70, 25, 60},
                        {20, 0, 10, 40, 35, 30, 50, 20},
                        {10, 20, 15, 35, 15, 15, 25, 15},
                        {15, 20, 10, 10, 15, 15, 15, 10},
                        {928, 2180, 945, 278, 1182, 896, 1329, 1397},
                        {295, 770, 440, 430, 315, 400, 379, 450} };
        // Note the use of ampl::StringArgs to pass an array of strings
        ampl::DataFrame df(2, ampl::StringArgs("NUTR", "FOOD", "amt"));
        df.setMatrix(nutrients, foods, amounts);

        std::string s = df.toString();
        printf("%s\n", s.c_str());

        s_ampl->setData(df);

        df = s_ampl->getData("{i in NUTR, j in FOOD} amt[i, j] / 2.0");
        s = df.toString();
        printf("%s\n", s.c_str());
    }
    
}

static void test_data_vector_declarations()
{
    s_ampl->reset();

    const char* model =
        "set INPUT_ROW;\n"
        "set INPUT_COL;\n"
        "param vectors {INPUT_ROW, INPUT_COL} >= 0.0;\n";

    save_data(TEST_FOLDER"test_data_vector_declarations.mod", model);
    s_ampl->read(TEST_FOLDER"test_data_vector_declarations.mod");

    int n = 10;
    double row_values[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    double col_values[] = { 1, 2, 3 };
    s_ampl->getSet("INPUT_ROW").setValues(row_values, 10);
    s_ampl->getSet("INPUT_COL").setValues(col_values, 3);

    ampl::DataFrame df = s_ampl->getSet("INPUT_ROW").getValues();
    std::string ds = df.toString();
    printf("%s\n", ds.c_str());

    ampl::DataFrame vectors(2, ampl::StringArgs("INPUT_ROW", "INPUT_COL", "vectors"));
    vectors.reserve(n * 3);
    for (int i = 1; i <= n; ++i)
    {
        double v = (double)i * 3;

        vectors.addRow(i, 1, v);
        vectors.addRow(i, 2, v + 1);
        vectors.addRow(i, 3, v + 2);
    }
    
    ds = vectors.toString();
    printf("%s\n", ds.c_str());

    try
    {
        s_ampl->setData(vectors);

        ampl::DataFrame df = s_ampl->getData("{i in INPUT_ROW, j in INPUT_COL} vectors[i, j] * 10.0");
        std::string s = df.toString();
        printf("%s\n", s.c_str());
    }
    catch (std::exception e)
    {
        printf("%s\n", e.what());
    }
}

static void test_simple_linear_least_squares()
{
    s_ampl->reset();

    const char* model =
        "set INPUT_ROW;\n"
        "set INPUT_TRIANGLES;\n"
        "set DIM = 1..3;\n"

        "param data_points {INPUT_ROW, DIM};\n"
        "param bary_centrics {INPUT_ROW, DIM};\n"
        "param triangle_points {INPUT_TRIANGLES, DIM};\n"

        "var move_points {INPUT_ROW, DIM};\n"

        "minimize energy: sum{i in INPUT_ROW} \n"
        " ((bary_centrics[i, 1] * move_points[i, 1] + \n"
        "  bary_centrics[i, 2] * triangle_points[i * 2 - 1, 1] + \n"
        "  bary_centrics[i, 3] * triangle_points[i * 2, 1] - data_points[i, 1]) * \n"
        " (bary_centrics[i, 1] * move_points[i, 1] + \n"
        "  bary_centrics[i, 2] * triangle_points[i * 2 - 1, 1] + \n"
        "  bary_centrics[i, 3] * triangle_points[i * 2, 1] - data_points[i, 1]) + \n\n"
        
        // "sum{i in INPUT_ROW} \n"
        " (bary_centrics[i, 1] * move_points[i, 2] + \n"
        " bary_centrics[i, 2] * triangle_points[i * 2 - 1, 2] + \n"
        " bary_centrics[i, 3] * triangle_points[i * 2, 2] - data_points[i, 2]) * \n"
        " (bary_centrics[i, 1] * move_points[i, 2] + \n"
        " bary_centrics[i, 2] * triangle_points[i * 2 - 1, 2] + \n"
        " bary_centrics[i, 3] * triangle_points[i * 2, 2] - data_points[i, 2]) + \n\n"
        
        // "sum{i in INPUT_ROW} \n"
        " (bary_centrics[i, 1] * move_points[i, 3] + \n"
        " bary_centrics[i, 2] * triangle_points[i * 2 - 1, 3] + \n"
        " bary_centrics[i, 3] * triangle_points[i * 2, 3] - data_points[i, 3]) * \n"
        " (bary_centrics[i, 1] * move_points[i, 1] + \n"
        " bary_centrics[i, 2] * triangle_points[i * 2 - 1, 3] + \n"
        " bary_centrics[i, 3] * triangle_points[i * 2, 3] - data_points[i, 3])); \n";

    save_data(TEST_FOLDER"test_simple_linear_least_squares.mod", model);
    s_ampl->read(TEST_FOLDER"test_simple_linear_least_squares.mod");

    const int DATA_COUNT = 10;
    const int TRIANGLE_POINT_COUNT = DATA_COUNT * 2;
    double row_values[DATA_COUNT] = { 0.f };
    double triangle_count_values[TRIANGLE_POINT_COUNT] = { 0.f };
    for (int i = 1; i <= DATA_COUNT; ++i)
        row_values[i - 1] = (double)i;
    for (int i = 1; i <= TRIANGLE_POINT_COUNT; ++i)
        triangle_count_values[i - 1] = (double)i;

    {
        s_ampl->getSet("INPUT_ROW").setValues(row_values, 10);
        s_ampl->getSet("INPUT_TRIANGLES").setValues(triangle_count_values, TRIANGLE_POINT_COUNT);
    }


    PoolAllocator allocator;
    RandomGen* rg = randomgen_create(&allocator, NULL);

    {
        ampl::DataFrame data_points(2, ampl::StringArgs("INPUT_ROW", "DIM", "data_points"));
        data_points.reserve(DATA_COUNT * 3);
        for (int i = 1; i <= DATA_COUNT; ++i)
        {
            double v = randomgen_get_uniform(rg);
            data_points.addRow(i, 1, v);

            v = randomgen_get_uniform(rg);
            data_points.addRow(i, 2, v);

            v = randomgen_get_uniform(rg);
            data_points.addRow(i, 3, v);
        }
        s_ampl->setData(data_points);
    }


    {
        ampl::DataFrame bary_centrics(2, ampl::StringArgs("INPUT_ROW", "DIM", "bary_centrics"));
        bary_centrics.reserve(DATA_COUNT * 3);
        for (int i = 1; i <= DATA_COUNT; ++i)
        {
            double v = randomgen_get_uniform(rg);
            bary_centrics.addRow(i, 1, v);

            v = randomgen_get_uniform(rg);
            bary_centrics.addRow(i, 2, v);

            v = randomgen_get_uniform(rg);
            bary_centrics.addRow(i, 3, v);
        }
        s_ampl->setData(bary_centrics);
    }

    {
        ampl::DataFrame triangle_points(2, ampl::StringArgs("INPUT_TRIANGLES", "DIM", "triangle_points"));
        triangle_points.reserve(TRIANGLE_POINT_COUNT * 3);
        for (int i = 1; i <= TRIANGLE_POINT_COUNT; ++i)
        {
            double v = randomgen_get_uniform(rg);
            triangle_points.addRow(i, 1, v);

            v = randomgen_get_uniform(rg);
            triangle_points.addRow(i, 2, v);

            v = randomgen_get_uniform(rg);
            triangle_points.addRow(i, 3, v);
        }
        s_ampl->setData(triangle_points);
    }

    {
        ampl::DataFrame move_points(2, ampl::StringArgs("INPUT_ROW", "DIM", "move_points"));
        move_points.reserve(DATA_COUNT * 3);
        for (int i = 1; i <= DATA_COUNT; ++i)
        {
            double v = randomgen_get_uniform(rg);
            move_points.addRow(i, 1, v);

            v = randomgen_get_uniform(rg);
            move_points.addRow(i, 2, v);

            v = randomgen_get_uniform(rg);
            move_points.addRow(i, 3, v);
        }
        
        s_ampl->setData(move_points);

        ampl::DataFrame df = s_ampl->getData("move_points");
        std::string s = df.toString();
        printf("%s\n", s.c_str());
    }

    s_ampl->solve();

    ampl::Variable v = s_ampl->getVariable("move_points");
    ampl::DataFrame vdf = v.getValues();
    std::string s = vdf.toString();
    printf("%s\n", s.c_str());

    ampl::DataFrame::Column col = vdf.getColumn("move_points.val");
    ampl::DataFrame::Column::iterator it = col.begin();
    for (;;)
    {
        if (it == col.end())
            break;

        double vx = it->dbl();
        ++it;

        double vy = it->dbl();
        ++it;

        double vz = it->dbl();
        ++it;

        printf("%f %f %f\n", vx, vy, vz);
    }

    
    randomgen_destroy(rg);
}

static void test_global_lls()
{
    s_ampl->reset();

    const char* model =
        "set M;\n"
        "set N;\n"
        "param A {M, N} default 0;\n"
        "param b {M, 1..3} default 0;\n"
        "var X {N, 1..3};\n"
        "minimize linear_least_squares:\n"
        "    sum{i in M, j in 1..3} (sum{c in N}(A[i, c] * X[c, j]) - b[i, j])^2"
        ;
        

    save_data(TEST_FOLDER"test_global_lls.mod", model);
    s_ampl->read(TEST_FOLDER"test_global_lls.mod");

    const int M_COUNT = 10;
    const int N_COUNT = 30;
    {
        
        double m_data[M_COUNT] = { 0, };
        double n_data[N_COUNT] = { 0, };

        for (int i = 1; i <= M_COUNT; ++i)
            m_data[i - 1] = (double)i;

        for (int i = 1; i <= N_COUNT; ++i)
            n_data[i - 1] = (double)i;

        s_ampl->getSet("M").setValues(m_data, M_COUNT);
        s_ampl->getSet("N").setValues(n_data, N_COUNT);

        /*
        ampl::DataFrame df = s_ampl->getData("{i in M, j in N} i + j");
        std::string s = df.toString();
        printf("%s\n", s.c_str());
        */
    }

    PoolAllocator allocator;
    RandomGen* rg = randomgen_create(&allocator, NULL);

    const bool SET_SPARSE_DATA = true;

    if (SET_SPARSE_DATA == true)
    {// set sparse data
        ampl::DataFrame matrix_a(2, ampl::StringArgs("M", "N", "A"));

        std::vector<double> rows;
        std::vector<double> cols;
        std::vector<double> values;
        rows.reserve(M_COUNT * N_COUNT);
        cols.reserve(M_COUNT * N_COUNT);
        values.reserve(M_COUNT * N_COUNT);

        for (int i = 1; i <= M_COUNT; ++i)
        {
            int js[] = { (int)(randomgen_get_uniform(rg) * N_COUNT + 1),
                (int)(randomgen_get_uniform(rg) * N_COUNT + 1),
                (int)(randomgen_get_uniform(rg) * N_COUNT + 1) };
            for (int j = 1; j <= N_COUNT; ++j)
            {
                bool zero = false;
                for (int k = 0; k < 3; ++k)
                {
                    if (js[k] == j)
                    {
                        zero = true;
                        break;
                    }
                }

                if (zero)
                    continue;

                rows.push_back(i);
                cols.push_back(j);
                values.push_back(randomgen_get_uniform(rg));
            }
        }

        matrix_a.setColumn("M", rows.data(), rows.size());
        matrix_a.setColumn("N", cols.data(), cols.size());
        matrix_a.setColumn("A", values.data(), values.size());

        s_ampl->setData(matrix_a);
        ampl::DataFrame df = s_ampl->getData("{i in M, j in N} A[i, j]");
        std::string s = df.toString();
        printf("%s\n", s.c_str());
    }

    if (SET_SPARSE_DATA == false)
    {// set dense data
        ampl::DataFrame matrix_a(2, ampl::StringArgs("M", "N", "A"));
        matrix_a.reserve(M_COUNT * N_COUNT);

        for (int i = 1; i <= M_COUNT; ++i)
        {
            int js[] = { (int)(randomgen_get_uniform(rg) * N_COUNT + 1), 
                (int)(randomgen_get_uniform(rg) * N_COUNT + 1), 
                (int)(randomgen_get_uniform(rg) * N_COUNT + 1)};

            for (int j = 1; j <= N_COUNT; ++j)
            {
                bool zero = false;
                for (int k = 0; k < 3; ++k)
                {
                    if (js[k] == j)
                    {
                        zero = true;
                        break;
                    }
                }

                double v = 0.0;
                if (!zero)
                    v = randomgen_get_uniform(rg);

                matrix_a.addRow(i, j, v);
            }
        }

        s_ampl->setData(matrix_a);

        ampl::DataFrame df = s_ampl->getData("{i in M, j in N} A[i, j]");
        std::string s = df.toString();
        printf("%s\n", s.c_str());
    }

    {
        ampl::DataFrame matrix_b(2, ampl::StringArgs("M", "DIM", "b"));
        matrix_b.reserve(M_COUNT * 3);

        for (int i = 1; i <= M_COUNT; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                double v = randomgen_get_uniform(rg);
                matrix_b.addRow(i, j, v);
            }
        }

        s_ampl->setData(matrix_b);

        /*
        ampl::DataFrame df = s_ampl->getData("{i in M, j in 1..3} B[i, j]");
        std::string s = df.toString();
        printf("%s\n", s.c_str());
        */
    }

    {
        ampl::DataFrame matrix_x(2, ampl::StringArgs("N", "DIM", "X"));
        matrix_x.reserve(N_COUNT * 3);

        for (int i = 1; i <= N_COUNT; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                double v = randomgen_get_uniform(rg);
                matrix_x.addRow(i, j, v);
            }
        }

        s_ampl->setData(matrix_x);

        /*
        ampl::DataFrame df = s_ampl->getData("{i in M, j in 1..3} X[i, j]");
        std::string s = df.toString();
        printf("%s\n", s.c_str());
        */
    }

    s_ampl->solve();

    ampl::Variable v = s_ampl->getVariable("X");
    ampl::DataFrame vdf = v.getValues();
    std::string s = vdf.toString();
    printf("%s\n", s.c_str());


    ampl::DataFrame::Column col = vdf.getColumn("X.val");
    ampl::DataFrame::Column::iterator it = col.begin();
    for (;;)
    {
        if (it == col.end())
            break;

        double vx = it->dbl();
        ++it;

        double vy = it->dbl();
        ++it;

        double vz = it->dbl();
        ++it;

        printf("%f %f %f\n", vx, vy, vz);
    }

    randomgen_destroy(rg);
}