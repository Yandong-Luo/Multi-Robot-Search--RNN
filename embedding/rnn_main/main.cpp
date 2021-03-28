/* GA-related headers */
#include <ga/ga.h>

/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>

#include <loop_functions/RNN_loop_functions/rnn_loop_function.h>

/****************************************/
/****************************************/

/*
 * Launch ARGoS to evaluate a genome.
 */
float LaunchARGoS(GAGenome& c_genome) {
   
   /* Convert the received genome to the actual genome type */
   GARealGenome& cRealGenome = dynamic_cast<GARealGenome&>(c_genome);
   /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance.
    * This variable is declared 'static' so it is created
    * once and then reused at each call of this function.
    * This line would work also without 'static', but written this way
    * it is faster. */
   static argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
   /* Get a reference to the loop functions */
   static CRNNLoopFunction& cLoopFunctions = dynamic_cast<CRNNLoopFunction&>(cSimulator.GetLoopFunctions());
   /*
    * Run 5 trials and take the worst performance as final value.
    * Performance in this experiment is defined as the distance from the light.
    * Thus, we keep the max distance found.
    */
   Real Fm = 0.0;
   for(size_t i = 0; i < 5; ++i) {
      /* Tell the loop functions to get ready for the i-th trial */
      cLoopFunctions.SetTrial(i);
      // std::cout<<"index:"<<i<<"\n";
      argos::LOG << "index #"<<i<< "...";
      /* Reset the experiment.
       * This internally calls also CEvolutionLoopFunctions::Reset(). */
      cSimulator.Reset();
      /* Configure the controller with the genome */
      cLoopFunctions.ConfigureFromGenome(cRealGenome);
      /* Run the experiment */
      cSimulator.Execute();
      /* Update performance */
      Fm = Fm + cLoopFunctions.Performance();
      // Fm = Min(Fm, cLoopFunctions.Performance());
      // Fm = Fm + cLoopFunctions.Performance();
   }
   Fm = Fm/5;
   std::cout<<"Fm:"<<Fm<<"\n";
   return Fm;
}

/*
 * Flush best individual
 */
void FlushBest(const GARealGenome& c_genome,
               size_t un_generation) {
   std::ostringstream cOSS;
   cOSS << "best_" << un_generation << ".dat";
   std::ofstream cOFS(cOSS.str().c_str(), std::ios::out | std::ios::trunc);
   cOFS << GENOME_SIZE // first write the number of values to dump
        << " "
        << c_genome    // then write the actual values
        << std::endl;
   std::cout<<"best_individual:"<<c_genome<<"\n";
}

/****************************************/
/****************************************/

int main(int argc, char** argv) {
   /*
    * Initialize GALIB
    */
   /* Create an allele whose values can be in the range [0,1] */
   GAAlleleSet<float> cAlleleSet(0.0f, 1.0f);
   
   /* Create a genome with 10 genes, using LaunchARGoS() to evaluate it */
   std::cout<<"GENOME_SIZE:"<<GENOME_SIZE<<"\n";
   GARealGenome cGenome(GENOME_SIZE, cAlleleSet, LaunchARGoS);
   /* Create and configure a basic genetic algorithm using the genome */
   GASteadyStateGA cGA(cGenome);
   // GASimpleGA cGA(cGenome);
   cGA.maximize();                     // the objective function must be maximized
   cGA.populationSize(100);              // population size for each generation
   cGA.pReplacement(0.80);
   cGA.nGenerations(500);              // number of generations
   cGA.pMutation(0.03f);               // prob of gene mutation
   cGA.pCrossover(0.7f);              // prob of gene crossover
   cGA.scoreFrequency(1);
   cGA.scoreFilename("evolution.dat"); // filename for the result log
   cGA.flushFrequency(1);              // log the results every generation
   /*
    * Initialize ARGoS
    */
   /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance */
   argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
   /* Set the .argos configuration file
    * This is a relative path which assumed that you launch the executable
    * from argos3-examples (as said also in the README) */
   cSimulator.SetExperimentFileName("experiments/PF_RNN.argos");

   cSimulator.LoadExperiment();

   /*
    * Launch the evolution, setting the random seed
    */
   cGA.initialize(12345);
   do {
      argos::LOG << "Generation #" << cGA.generation() << "...";
      cGA.step();
      argos::LOG << "done.";
      if(cGA.generation() % cGA.flushFrequency() == 0) {
         argos::LOG << "   Flushing...";
         /* Flush scores */
         cGA.flushScores();
         /* Flush best individual */
         FlushBest(dynamic_cast<const GARealGenome&>(cGA.statistics().bestIndividual()),
                   cGA.generation());
         // FlushBest(dynamic_cast<const GARealGenome&>(cGA.statistics().offlineMax()),
         //           cGA.generation());
         argos::LOG << "done.";
      }
      LOG << std::endl;
      LOG.Flush();
   }
   while(! cGA.done());

   /*
    * Dispose of ARGoS stuff
    */
   cSimulator.Destroy();

   /* All is OK */
   return 0;
}

/****************************************/
/****************************************/
