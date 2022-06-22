#include <future>
#include <memory>
#include <utility>

#include "kimera-vio/pipeline/Pipeline-definitions.h"

int main(int argc, char* argv[]){
    
    VIO::VioParams vio_params("folder/path");
    vio_params.print();
}
