%% METR4202 Lab 3 Random Order Generator
% METR4202 -- Lab 3: Systems & Controls -- Raise. The Bar
% -- Due: November 12, 2014 --
%
% Function picks a random set of coffee, tea, etc. menu items
% as specified in the "The Menu" section
%
function Order=Lab3RandOrder(NumberOfDrinks, AllowEspresso, AllowUrgent)

%% DEFINE Program Parameters
% Probability for Expresso and Urgency (from [0 to 1])
PROB_ESPRESSO = 0.5;
PROB_URGENT = 0.3333;
MAX_NUM_DRINKS = 5;
MAX_CUPSIZE=2;
MAX_COFFEE=2;
MAX_TEA=2;
MAX_SUGAR=3;

%% Check for no argument case
if nargin == 0
    NumberOfDrinks=0;
    AllowEspresso=1;
    AllowUrgent=1;
end

%%  Determine Number of Drinks
% If no NumberOfDrinks arguments (NumberofDrink is zero)
% then randomly select the number of drinks
if (~NumberOfDrinks)
    NumberOfDrinks=PickANumber(MAX_NUM_DRINKS);  
end


%% Initialize
Order=cell( 1+NumberOfDrinks, 1);
Order{1}=[NumberOfDrinks, 0, 0, 0, 0, 0];

%% Process Each Drink
% Select Drinks
for ii=1:NumberOfDrinks
    Order{ii+1}=RandomDrink;
end

%% Display Order Vector
csvwrite('./orders', Order);
celldisp(Order)

%% Nested function to make a drink
    function drink=RandomDrink()
        % RANDOMDRINK returns a random drink order
        % [drink] =  [ CupSize, Ncoffee, Ntea, Nsugar, Nespresso, IsUrgent], with:
        % CupSize: Medium [1] or Large [2]
        % Ncoffee: Number of coffee satchels needed [0, 2]
        % Ntea: Number of tea bags needed [0, 2].  Note that if Ncoffee is >0, then Ntea is 0.  That is a drink will not ask for coffee and tea together.
        % Nsugar: Number of sugars needed [0, 3]
        % NEspresso [optional]: If the order is for a “fancy” espresso [0, 1].  Note: That Nespresso is 0 or 1 and if is 1, then Ntea and Ncoffee will be 0, that is a drink is either only a coffee, tea, or espresso.   As “espresso” is an optional
        % IsUrgent [optional] = An indicator that the order is labeled “urgent” and that it is needed this first [0, 1].  If an order consists of multiple drinks with an IsUrgent of 1, it may fill these IsUrgent drinks in any order it chooses such that they are delivered before non-urgent drinks (IsUrgent=0).
        
        % Initialize
        CupSize=0; Ncoffee=0; Ntea=0; Nsugar=0; Nespresso=0; IsUrgent=0;
        
        % Select The Items...
        CupSize=PickANumber(MAX_CUPSIZE);  % Medium [1] or Large [2]
        
        Ncoffee=PickANumber(MAX_COFFEE+1)-1;  % Number of coffee satchels needed [0, 1, 2].  Pick 1,2,or 3, then substract 1
        
        if(~Ncoffee) Ntea=PickANumber(MAX_TEA); end % Number of tea satchels needed [1, 2];  Pick 1 or 2 (not zero ? if we're asking for tea, then there is no coffee)
        % Instead we could have:
        % Ntea=PickANumber(MAX_TEA)-1
        % %Check the Tea/Coffee Conditional
        % if Ncoffee>0
        %     Ntea=0;
        % end
        
        Nsugar=PickANumber(MAX_SUGAR)-1;  % Number of coffee satchels needed [0, 1, 2, 3];
        
        % Handle Espresso(s) (if allowed)
        % Nested if loop for readability
        if AllowEspresso
            if Ncoffee
                if(SelectThis(PROB_ESPRESSO))
                    Nespresso=1;
                    Ncoffee=0;
                else
                    Nespresso=0;
                end
            end
        end
        
        
        % Handle Urgent (if allowed)
        % Nested if loop for readability
        if AllowUrgent
            if(SelectThis(PROB_URGENT))
                IsUrgent=1;
            else
                IsUrgent=0;
            end
        end
                
        % PLACE in vector
        drink=[CupSize, Ncoffee, Ntea, Nsugar, Nespresso, IsUrgent];
    end
end




%% Sub-function to randomly select an integer number
function rndnumber=PickANumber(from1toN)
% PickANumber returns a random number from 1 to the integer value given
rndnumber=ceil(rand(1)*from1toN);
%  rndnumber=builtin('_paren', randperm(round(from1toN)), 1);
% This is an inline form of temp=randperm(5); NumberOfDrinks=temp(1);
% The round is to check for integers (in case others want to use this
% (though MATLAB does not allow for localfunctions to be called from
% other functions or the command line)
end


%% Sub-function to select a case, relative to a given probability factor
function output_switch=SelectThis(THRESHOLD)
% SelectThis with output a TRUE based on a given threshold
randnum=rand(1);
if (randnum)<=THRESHOLD
    output_switch=true;
else
    output_switch=false;
end
end