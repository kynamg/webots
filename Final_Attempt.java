import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Random;
public class Final_Attempt
{
	private static int max_cities=16; 							
	private static int max_population=30;						
	private static int population_size=30;						
	private static int maxgenerations= 60;						
	private static Random random= new Random();					
	private static int numberofcities=8;						
	private static double[][] distance;							
	private static int[][] currentpopulation;					
	private static int[][] intermediatepopulation;				
	private static double[] fitness;							
	private static int bestsolutionsofar;						
	private static double bestfitnesssofar=0;						 
	private static int[] referencesolution;						
	private static String DISTANCE_FILENAME="distances.txt";
	private static int elitism=1;
	private static double mutationrate1=0.1;
	private static double mutationrate2=0.5;
	private static double mutationrate3=0.8;
	private static double fitness_percentage;
	
	public static void main(String[] args)
	{
		int generation=1;
		distance=new double[max_cities][max_cities];
		currentpopulation= new int[max_population][max_cities];
		intermediatepopulation= new int[max_population][max_cities];
		fitness= new double[max_population];
		referencesolution=new int[max_cities];
		readthedistancematrix();
		initialisethepopulation();
		evaluatethepopulation();
		while(generation<maxgenerations)
		{
			System.out.println("GENERATION:"+ generation);
			producethenextgeneration();
			evaluatethepopulation();
			generation++;
		}
		
		
	}
	private static int[] crossover(int[] father,int[] mother)
	{
		int[] child=new int[max_cities];
		int[] father_copy= new int[max_cities];
		int[] mother_copy= new int[max_cities];
		int crossover_point=getRandomNumberInBetween(0,father.length-1);
		for(int i=0;i<father.length;i++)
		{
			if(i<crossover_point)
			{
				child[i]=father[i];
				father[i]=0;
				for(int p=0;p<mother.length;p++)
				{
					int m=mother[p];
					if(m==child[i])
					{
						mother[p]=0;
					}
				}
				//for(int p=0;p<father.length;p++)
				//{
					//int c=father[p];
					//if(c!=child[i])
					//{
						//father_copy[p]=father[p];
					//}
					//int m=mother[p];
					//if(m!=child[i])
					//{					
						//mother_copy[p]=mother[p];
					//}
				//}
				//for(int p=0;p<father_copy.length;p++)
				//{
					//father[p]=father_copy[p];
					//mother[p]=mother_copy[p];
				//}
			}
			else
			{
				child[i]=mother[i];
				mother[i]=0;
				for(int p=0;p<mother.length;p++)
				{
					int m=father[p];
					if(m==child[i])
					{
						father[p]=0;
					}
				}
//				for(int p=0;p<mother.length;p++)
//				{
//					int c=father[p];
//					if(c!=child[i])
//					{
//						father_copy[p]=father[p];					
//					}					
//					int m=mother[p];
//					if(m!=child[i])
//					{
//						mother_copy[p]=mother[p];
//					}
//				}
//				for(int p=0;p<mother_copy.length;p++)
//				{
//					father[p]=father_copy[p];
//					mother[p]=mother_copy[p];
//				}
			}
		}
		return child;
	}
	private static void producethenextgeneration()
	{
		int[] child= new int[max_cities];
		int[] father=new int[max_cities];
		int[] mother=new int[max_cities];
		int population_1,population_2,city,new_population;
		if(elitism==1)
		{
			for(city=0;city<numberofcities;city++)
			{
				intermediatepopulation[0][city]=currentpopulation[bestsolutionsofar][city];
			}
		}
		for(new_population=0;new_population<population_size;new_population++)
		{
			population_1=roulette_wheel_selection();
			population_2=roulette_wheel_selection();
			for(int i=0;i<numberofcities;i++)
			{
				father[i]=currentpopulation[population_1][i];
			}
			for(int i=0;i<numberofcities;i++)
			{
				mother[i]=currentpopulation[population_2][i];
			}
			child=crossover(father,mother);
			child=mutation(child);
			for(city=0;city<numberofcities;city++)
			{
				intermediatepopulation[new_population][city]=child[city];
			}
		}
		for(new_population=0;new_population<population_size;new_population++)
		{
			for(city=0;city<numberofcities;city++)
			{
				currentpopulation[new_population][city]=intermediatepopulation[new_population][city];
			}
		}
	}
	private static int roulette_wheel_selection()
	{
		double r=getRandomNumberFrom(0,10000);
		double r_prob=r/10000;
		int population;
		double tot_fitness=0;
		double sum_prob=0;
		for(population=0;population<population_size;population++)
		{
			tot_fitness=tot_fitness+fitness[population];
		}
		for(population=0;population<population_size;population++)
		{
			sum_prob=sum_prob+fitness[population]/tot_fitness;
			if(sum_prob>r_prob)
			{
				return population;
			}
		}
		return 0;
	}
	private static void evaluatethepopulation()
	{
		int population, city, city1, city2;
		double total_distance;
		for(population=0;population<population_size;population++)
		{
			total_distance=0;
			for(city=0;city<numberofcities;city++)
			{
				city1=currentpopulation[population][city];
				city2=currentpopulation[population][(city+1)%numberofcities];
				total_distance+=distance[city1][city2];
			}
			if(total_distance==0)
			{
				System.out.println("Not possible");
				System.exit(0);
			}
			fitness[population]=1.0/total_distance;
			if(fitness[population]>= bestfitnesssofar)
			{
				bestfitnesssofar=fitness[population];
				bestsolutionsofar=population;
			}
		}
		for(population=0;population<population_size;population++)
		{
			if(population==bestsolutionsofar)
			{
				System.out.println("best so far: ");
			}
			for(city1=0;city1<numberofcities;city1++)
			{
				System.out.print(currentpopulation[population][city1]+" ");
			}
			fitness_percentage = Math.round(((fitness[population])*100)*100000d)/100000d;
			System.out.println("- "+fitness_percentage+"%"+"(distance: "+Math.round(1.0/fitness[population])+")\n");
		}
		System.out.println("\n");
	}
	private static void initialisethepopulation()
	{
		int population,length,thiscity;
		for(population=0;population<population_size;population++)
		{
			for(int city=0;city<numberofcities;city++)
			{
				referencesolution[city]=city;
			}
			length=numberofcities;
			for(int city=0;city<numberofcities;city++)
			{
				thiscity= getRandomNumberInBetween(0,length);
				currentpopulation[population][city]=referencesolution[thiscity];
				referencesolution[thiscity]=referencesolution[length-1];
				length--;
			}
		}
		
		
	}
	private static int getRandomNumberInBetween(int min,int max)
		{
			return random.nextInt(max-min)+min;
		}
		
		public static int getRandomNumberFrom(int min,int max)
		{
			return getRandomNumberInBetween(min,max+1);
		}
	private static int[] mutation(int[] child)
	{
		int mutation=getRandomNumberInBetween(0,10);
		if((mutation>mutationrate1*10)&&(mutation<mutationrate2*10))
		{
			int city = getRandomNumberInBetween(0, numberofcities);
			int local = child[city];
			child[city]=child[(city+1)%numberofcities];
			child[(city+1)%numberofcities]=local;
		}
		if((mutation>mutationrate2*10)&&(mutation<mutationrate3*10))
		{
			int city = getRandomNumberInBetween(0, numberofcities);
			int local = child[city];
			child[city]=child[(city+2)%numberofcities];
			child[(city+2)%numberofcities]=local;
		}
		if(mutation>mutationrate3*10)
		{
			int city = getRandomNumberInBetween(0, numberofcities);
			int local = child[city];
			child[city]=child[(city+4)%numberofcities];
			child[(city+4)%numberofcities]=local;
		}
		return child;
	} 
	private static void readthedistancematrix() 
	{
		int city1,city2;
		BufferedReader readbuffer=null;
		String strread;
		String splitarray[];
		double inputnumberdouble;
		try
		{
			readbuffer=new BufferedReader(new FileReader(DISTANCE_FILENAME));
			for(city1=0;city1<numberofcities;city1++)
			{
				strread=readbuffer.readLine();
				splitarray=strread.split("\t");
				for(city2=0; city2<numberofcities;city2++)
				{
					inputnumberdouble=Double.parseDouble(splitarray[city2]);
					distance[city1][city2]= inputnumberdouble;
				}
			}
		}catch(Exception e)
		{
			System.out.println(e);
			System.exit(0);
		}
		
	}
}
