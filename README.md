# SAFFRON:<br><sub>🚦Store-And-Forward model toolbox For urban ROad Network signal control in MATLAB</sub>

[![GitHub release](https://img.shields.io/github/release/Naereen/StrapDown.js.svg)](https://github.com/decenter2021/SAFFRON/releases)
[![MIT license](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/decenter2021/SAFFRON/LICENSE)
[![DOI:[not_published_yet]](https://zenodo.org/badge/DOI/not_published_yet.svg)](https://doi.org/not_published_yet.svg)

## 🎯 Features
- Collection of open-source and well documented tools for the  **synthesis, analysis, and simulation** of **store-and-forward** models of **urban road networks** in MATLAB
- Full model of the urban traffic network of the city center of **Chania, Greece**
- Implementation of **state-of-the-art** traffic responsive signal control strategies

***
## 🚀 Index

- [Description](#-description)
- [Authors](#-authors)
- [Contact](#-contact)
- [Documentation](#-documentation)
- [Contributing to SAFFRON](#-contributing-to-saffron)
- [Lincense](#-license)
- [References](#-references)

***

## 💡 Description
<p align="justify">
  The SAFFRON toolbox is introduced to <b>synthesize, analyze, and simulate store-and-forward</b> based strategies for the signal control problem in congested urban road networks in <b>MATLAB</b>. SAFFRON is tecnhically described in 
</p>

<a href="">Pedroso, L., Batista, P., Papageorgiou, M. and Kosmatopoulos, E. [not published yet]</a>
  
If you use SAFFRON, reference the publication above.

SAFFRON allows to:
- Seamlessly simulate store-and-forward based signal control strategies
- Apply novel solutions to a meaningful model that is publicly available and, hence, can be reproduced
- Compare novel solutions with other control strategies with little effort

Low-level thorough [documentation](#-documentation) is provided in the repository page, as well as in the source files.

The community is encouraged to [contribute](#-contributing-to-saffron) to SAFFRON with suggestions, additions, and the implementation of signal control strategies.

***

## ✍🏼 Authors 
Leonardo Pedroso<sup>1</sup> <a href="https://github.com/leonardopedroso"><img src="https://github.githubassets.com/images/modules/logos_page/GitHub-Mark.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a> <a href="https://scholar.google.com/citations?user=W7_Gq-0AAAAJ"><img src="https://cdn.icon-icons.com/icons2/2108/PNG/512/google_scholar_icon_130918.png" style="width:1em;margin-right:.5em;"></a> <a href="https://orcid.org/0000-0002-1508-496X"><img src="https://orcid.org/sites/default/files/images/orcid_16x16.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a><br>
Pedro Batista<sup>1</sup> <a href="https://scholar.google.com/citations?user=6eon48IAAAAJ"><img src="https://cdn.icon-icons.com/icons2/2108/PNG/512/google_scholar_icon_130918.png" style="width:1em;margin-right:.5em;"></a> <a href="https://orcid.org/0000-0001-6079-0436"><img src="https://orcid.org/sites/default/files/images/orcid_16x16.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a><br>
Markos Papageorgiou<sup>2</sup> <a href="https://scholar.google.com/citations?user=sr8TOTQAAAAJ"><img src="https://cdn.icon-icons.com/icons2/2108/PNG/512/google_scholar_icon_130918.png" style="width:1em;margin-right:.5em;"></a> <a href="https://orcid.org/0000-0001-5821-4982"><img src="https://orcid.org/sites/default/files/images/orcid_16x16.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a><br>
Elias Kosmatopoulos<sup>3</sup> <a href="https://scholar.google.com/citations?user=LIRR8B8AAAAJ"><img src="https://cdn.icon-icons.com/icons2/2108/PNG/512/google_scholar_icon_130918.png" style="width:1em;margin-right:.5em;"></a> <a href="https://orcid.org/0000-0002-3735-4238"><img src="https://orcid.org/sites/default/files/images/orcid_16x16.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a><br>
<sub>*<sup>1</sup>Institute for Systems and Robotics, Instituto Superior Técnico, Universidade de Lisboa, Portugal<br>
  <sup>2</sup>Dynamic Systems and Simulation Laboratory, Technical University of Crete, GR-73100 Chania, Greece<br>
  <sup>3</sup>Department of Electrical and Computer Engineering, Democritus University of Thrace, Xanthi, Greece<br>*</sub>





***

## 📞 Contact
SAFFRON toolbox is currently maintained by Leonardo Pedroso (<a href="mailto:leonardo.pedroso@tecnico.ulisboa.pt">leonardo.pedroso@tecnico.ulisboa.pt</a>).


***

## 📖 Documentation
The documentation is divided into the following categories:
- [Model synthesis](#model-synthesis)
- [Utilities](#utilities)
  * [Network properties](#network-properties)
  * [Performance metrics](#performance-metrics)
  * [Quadratic continuous knapsack solver](#quadratic-continuous-knapsack-solver)
- [Simulation script](#simulation-script)
- [Chania urban road network](#chania-urban-road-network)
- [Example](#example)

### Model synthesis
<p align="justify">
A store-and-forward urban road network can be synthesized seamlessly with SAFFRON by filling a custom spreadsheet. A template is provided in a <a href=https://github.com/decenter2021/SAFFRON/tree/master/ModelTemplate>subfolder</a> of the repository. The following data is input in the spreadsheet:
</p>

- the number of junctions ![$J$](https://render.githubusercontent.com/render/math?math=\color{gray}J), links ![$Z$](https://render.githubusercontent.com/render/math?math=\color{gray}Z), and stages ![$S$](https://render.githubusercontent.com/render/math?math=\color{gray}S), control cycle ![$C$](https://render.githubusercontent.com/render/math?math=\color{gray}C), simulation cycle ![$T$](https://render.githubusercontent.com/render/math?math=\color{gray}T), and the upstream gating parameter ![$c_ug$](https://render.githubusercontent.com/render/math?math=\color{gray}c_{ug})
- the lost time and number of stages in each junction 
- the capacity, saturation flow, number of lanes, initial number of vehicles, and demand flow for each link
- the minimum green time and historic green time of each stage
- the stage matrix ![$\mathbf{S}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{S}), *i.e.*, a table that indicates which links have right of way (r.o.w.) for each stage
- the turning rates matrix ![$\mathbf{T}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{T}), *i.e.*, a table that indicates the probability of turning into the links of the network on the exit of a certain link, and the exit rate of all links ![$\mathbf{t_0}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{t_0})

<p align="justify">
Each table of the spreadsheet has to copied and pasted to <i>txt</i> whose names are indicated next to each table in the spreadsheet. These <i>txt</i> files have to be enclosed in a folder with the name of the urban road model. Template <i>txt</i> files and an example of this procedure are provided in a <a href=https://github.com/decenter2021/SAFFRON/tree/master/ModelTemplate>subfolder</a> of the repository.
</p>

The model is loaded into MATLAB making use of the following command 
```
>> model = SFMSynthesis("directory")
```
<p align="justify">
where <tt>directory</tt> is the enclosing folder of the <i>txt</i> files and <tt>model</tt> is a <i>MATLAB struct</i> object that characterizes the urban road network. The command above also saves the model object, as well as the raw input tables, in the file <i>data.txt</i> in the model folder. Thus afterwards the models can also be loaded using</p>

```
>> model = load("directory/data.mat")
```

The **fields** of store-and-forward model object *struct* are 

| Field  | Description | Notation |
| -- | -- |------------|
| <tt>J</tt> | Number of junctions | ![$J$](https://render.githubusercontent.com/render/math?math=\color{gray}J)|
| <tt>Z</tt> | Number of junctions | ![$L$](https://render.githubusercontent.com/render/math?math=\color{gray}Z)|
| <tt>nStages</tt> | Number of stages | ![$L$](https://render.githubusercontent.com/render/math?math=\color{gray}S)|
| <tt>C</tt> | Control cycle (s) | ![$C$](https://render.githubusercontent.com/render/math?math=\color{gray}C)|
| <tt>c</tt> | Upstream gating parameter | ![$c_{ug}$](https://render.githubusercontent.com/render/math?math=\color{gray}c_{ug})|
| <tt>Tsim</tt> | Simulation cycle (s) | ![$T$](https://render.githubusercontent.com/render/math?math=\color{gray}T)|
| <tt>lostTime</tt> | Column vector of lost times (s) in each junction  | ![$\mathrm{col}(L_1,\ldots,L_J)$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathrm{col}(L_1,\ldots,L_J))|
| <tt>nStagesJunction</tt> | Column vector of number of stages of each junction | <img src="https://render.githubusercontent.com/render/math?math=\color{gray}\mathrm{col}(&#124\mathcal{F}_1&#124,\ldots,&#124\mathcal{F}_S&#124))" alt="$\mathrm{col}(&#124\mathcal{F}_1&#124,\ldots,&#124\mathcal{F}_S&#124)$">|
| <tt>capacity</tt> | Column vector of capacity of each link (veh) | ![$\mathrm{col}(x_{1,\text{max}},\ldots,x_{Z,\text{max}})$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathrm{col}(x_{1,\text{max}},\ldots,x_{Z,\text{max}}))|
| <tt>saturation</tt> | Column vector of saturation flow of each link (veh/s) | ![$\mathrm{col}(S_1,\ldots,S_Z)$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathrm{col}(S_1,\ldots,S_Z))|
| <tt>lanes</tt> | Column vector of number of lanes in each link | - |
| <tt>x0</tt> | Initial number of vehicles of each link (veh) | ![$\mathbf{x}(0)$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{x}(0))|
| <tt>d</tt> | Column vector of demand on each link (veh/s) | ![$\mathrm{col}(d_1,\ldots,d_Z)$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathrm{col}(d_1,\ldots,d_Z))|
| <tt>gmin</tt> | Column vector of minimum green time of each stage (s) | ![$\mathrm{col}(g_{1,min},\ldots,g_{S,min})$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathrm{col}(g_{1,min},\ldots,g_{S,min}))|
| <tt>gN</tt> |  Column vector of historic green time of each stage (s) | ![$\mathbf{g_N}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{g_N})|
| <tt>T</tt> | Turning rates matrix | ![$\mathbf{T}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{T})|
| <tt>t0</tt> | Exit rates vector | ![$\mathbf{t_0}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{t_0})|
| <tt>S</tt> | Stage matrix | ![$\mathbf{S}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{S})|
| <tt>junctions</tt> | Cell array indexed by junction number that contains the number of the stages associated with that junction (see example below) | -|
| <tt>links</tt> | <img src="https://render.githubusercontent.com/render/math?math=\color{gray}Z\times 2" alt="$Z\times 2$"> matrix whose rows are indexed by link number and the corresponding columns are the origin and destination junctions respectively, *i.e.*, it is the ordered list of edges of the network graph (see example below) | - |
| <tt>inLinks</tt> | Column vector of link indices that originate from outside the network | - |
| <tt>notInLinks</tt> | Column vector of link indices that do not originate from outside the network | - |
| <tt>A</tt> | State-space matrix ![$\mathbf{A}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{A}) of the store-and-forward model of the network | ![$\mathbf{A}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{A})|
| <tt>Bu</tt> | State-space matrix ![$\mathbf{A}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B}) of the store-and-forward model in equation (4) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#-references) | ![$\mathbf{B_u}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B_u})|
| <tt>BG</tt> | State-space matrix ![$\mathbf{A}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B}) of the store-and-forward model in equation (7) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#-references) | ![$\mathbf{B_G}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B_G})|
| <tt>Bg</tt> | State-space matrix ![$\mathbf{A}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B}) of the store-and-forward model in equation (8) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#-references) | ![$\mathbf{B_g}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B_g})|
| <tt>Bu_sim</tt> | State-space matrix ![$\mathbf{A}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B}) of the store-and-forward model in equation (4) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#-references) discretized with simulation cycle ![$T$](https://render.githubusercontent.com/render/math?math=\color{gray}T) | - |
| <tt>BG_sim</tt> | State-space matrix ![$\mathbf{A}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B}) of the store-and-forward model in equation (7) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#-references) discretized with simulation cycle ![$T$](https://render.githubusercontent.com/render/math?math=\color{gray}T) | - |
| <tt>Bg_sim</tt> | State-space matrix ![$\mathbf{A}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B}) of the store-and-forward model in equation (8) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#-references) discretized with simulation cycle ![$T$](https://render.githubusercontent.com/render/math?math=\color{gray}T) | - |
| <tt>C_z</tt> | State-space matrix ![$\mathbf{C}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{C}) of the store-and-forward model of the network | ![$\mathbf{C}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{C})|
| <tt>E_DTUC_psi</tt> | Sparsity matrix of the controller gain for the decentralized control strategy DTUC with decentralized configuration ![$\mathbf{\Psi}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{\Psi}) proposed in [(Pedroso and Batista, 2021)](#-references) | ![$\mathbf{E_{\Psi}}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{E_{\Psi}})|
| <tt>E_DTUC_phi</tt> | Sparsity matrix of the controller gain for the decentralized control strategy DTUC with decentralized configuration ![$\mathbf{\Phi}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{\Phi}) proposed in [(Pedroso and Batista, 2021)](#-references) | ![$\mathbf{E_{\Phi}}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{E_{\Phi}})|
| <tt>E_D2TUC_psi</tt> | Sparsity matrix of the controller gain for the decentralized control strategy D2TUC with decentralized configuration ![$\mathbf{\Psi}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{\Psi}) proposed in [(Pedroso and Batista, 2021)](#-references) | ![$\mathbf{E_{\Psi}}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{E_{\Psi}})|
| <tt>E_D2TUC_phi</tt> | Sparsity matrix of the controller gain for the decentralized control strategy D2TUC with decentralized configuration ![$\mathbf{\Phi}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{\Phi}) proposed in [(Pedroso and Batista, 2021)](#-references) | ![$\mathbf{E_{\Phi}}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{E_{\Phi}})|

>Example: *Load the Chania urban road network provided in SAFFRON*
>```
>>> chania = SFMSynthesis('ChaniaUrbanRoadModel'); % Load Chania, Greece urban road network
>>> chania.S                                       % Get number of stages
> ans = 
>     42
>>> chania.nStagesJunction(4)                      % Get number of stages associated with junction 4
> ans =
>     3
>>> chania.junctions{4}                            % Get the indices of the stages associated with junction 4
> ans =
>     8     9    10
>>> chania.links(13,:)                             % Get origin and destination junction of link 13
> ans =
>     5     4                                       % Link 13 goes from junction 5 towards junction 4
  
### Utilities

#### Network properties
<p align="justify">
  It is possible to check if the traffic network is <b>open</b> and if it has a <b>minimum complete stage strategy</b>. These properties are defined with rigour in <a href="#-references">(Pedroso and Batista, 2021)</a>. They are closely related with the controllability of the store-and-forward model.
</p>

> A traffic network is said to be **open** if there is a directed walk starting at every link which a vehicle may follow to exit the network with non-zero probability. [(Pedroso and Batista, 2021)](#-references)<br>

> A traffic network is said to be **feasible** if it is **finite** and **open**. [(Pedroso and Batista, 2021)](#-references)

To check if the store-and-forward model <tt>model</tt> is open, thus fesible, the following command is used
```
>> flag = isOpen(model);
```
which outputs a boolean.

To check if the store-and-forward model <tt>model</tt> has a minimum complete stage strategy, the following command is used
```
>> flag = isMinimumComplete(model);
```
which outputs a boolean.

>Example: *Check if the Chania urban road network provided in SAFFRON is open and if it has a minimum complete stage strategy*
>```
>>> chania = SFMSynthesis('ChaniaUrbanRoadModel'); % Load Chania, Greece urban road network
>>> flag = isOpen(chania)                          % Check if it is open
> flag  = 
>   logical
>     1
>>> flag = isMinimumComplete(chania)               % Get it has a minimum complete stage strategy
> flag  = 
>   logical
>     1

#### Performance metrics


The performance metrics total time spent (TTS) <br>
<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=\color{gray}\mathrm{TTS} = T\sum_{k}\sum_{z=1}^Zx_z(k)" alt="$\mathrm{TTS} = C\sum_{k}\sum_{z=1}^Zx_z(k)$">
</p><br>
and relative queue balance (RQB) <br>
<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=\color{gray}\mathrm{RQB} = \sum_{k}\sum_{z=1}^Z\frac{x_z^2(k)}{x_{z,max}}" alt="$\mathrm{RQB} = \sum_{k}\sum_{z=1}^Z\frac{x_z^2(k)}{x_{z,max}}$">
</p><br>
<p align="justify">
introcuced in <a href="#-references">(Aboudolas, Papageorgiou, and Kosmatopoulos, 2009)</a> can be computed seamlessly with SAFFRON. 
Let <tt>xNL</tt> be a <img src="https://render.githubusercontent.com/render/math?math=\color{gray}Z\times N_{sim}" alt="$Z\times N_{sim}$"> array, where <img src="https://render.githubusercontent.com/render/math?math=\color{gray}N_{sim}" alt="$N_{sim}$"> is the number of cycles that were simulated of a model <tt>model</tt> with the nonlinear dynamiccs (See <a href="simulation-script">Simulation script</a> for a template to simulate the nonlinear dynamics and <a href="example">Example</a> for an example). The <img src="https://render.githubusercontent.com/render/math?math=\color{gray}k" alt="$k$">-th row of <tt>xNL</tt>, <i>i.e.</i> <tt>xNL(:,k)</tt>, is the column vector of the link occupancy at time <img src="https://render.githubusercontent.com/render/math?math=\color{gray}t = (k-1)T" alt="$t = (k-1)T$">. The command
</p>

```
>> [TTS,RQB] = SFMMetrics(model,xNL);
```

outputs the <tt>TTS</tt> (in veh h) and the <tt>RQB</tt> (in veh).


#### Quadratic continuous knapsack solver
<p align="justify">
  The <b>quadratic continuous knapsack problem</b> often arises in a post-processing stage of a continous traffic signal control policy to <b>allocate the green times among the stages</b>. The command
</p>

```
>> x = knapsack(a,b,c,d);
```

<p align="justify">
where <tt>a</tt>,<tt>b</tt>, and <tt>d</tt> are column vectors and <tt>c</tt> is a scalar, outputs the solution. For more details see <a href="#-references">(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)</a>. The algorithm that is implemented is poposed in <a href="#-references">(Helgason, Kennington, and Lall, 1980)</a>, it is detailled in the context of traffic signal control in <a href="#-references">(Diakaki, 1999)</a>.
  </p>

>Example: *Quadratic continuous knapsack problem*
>```
>>> a = [0;-1;0];
>>> b = [4;2;1];
>>> c = 5;
>>> d = [1;1;1];
>>> x = knapsack(a,b,c,d)
>x =
>   2.5000
>   1.5000
>   1.0000 

### Simulation script

<p align="justify">
SAFFRON toolbox also provides <tt>simulation_template.m</tt> template file for the simulation of a traffic signal control policy in MATLAB making use of the nonlinear model with upstream gating that is proposed in <a href="#-references">(Aboudolas, Papageorgiou, and Kosmatopoulos, 2009)</a>. For more tecnhical details see <a href="#-references">(Pedroso, Batista, Papageorgiou Kosmatopoulos, 2022)</a>.
</p>

There are 7 main 

##### Import urban road network model

```Matlab
% Import Chania network
model = SFMSynthesis("ChaniaUrbanRoadModel");
```

##### Set different initial conditions and demand if desired

```Matlab
% Chania urban road model has initial condictions and demands for a
% scenario. These can be used as a benchmark. To test different initial
% conditions and demand scenarios simply make modifications to model.x0 and
% model.d
% For example:
rng(1); % Seed for consistency
% Set initial conditions
model.x0 = 0.5*rand(model.Z,1).*model.capacity;
% Set demand 
model.d = zeros(model.Z,1);
model.d(model.inLinks) = 0.1*model.saturation(model.inLinks).*rand(length(model.inLinks),1);
model.d(model.notInLinks) = 0.01*model.saturation(model.notInLinks).*rand(length(model.notInLinks),1);
```

### Chania urban road network

<p align="justify">
The model of the urban traffic network of the city center of Chania, Greece is provided. This model has been extensively used by the DSSLab <a href="#-references">(Diakaki, 1999)</a>, <a href="#-references">(Aboudolas, Papageorgiou, Kosmatopoulos, 2009)</a>.
</p>

The Chania urban traffic network, consists of 16 signalized junctions and 60 links

<img src="https://user-images.githubusercontent.com/40807922/161239424-79df7877-7ba5-45b5-b4f5-8a0165f4f31f.png" alt="ChaniaUrbanRoadModelGraph"/>

The model data in enclosed in the folder <a href="https://github.com/decenter2021/SAFFRON/tree/master/ChaniaUrbanRoadModel"><tt>ChaniaUrbanRoadModel</tt></a> includes
- model paramenters in a spreadsheet that follows the provided [template](#model-synthesis) and associated *txt* files 
- model paramenters in the MAT-fite <tt>data.mat</tt>
- the image above of the topology of the network

To load the Chania urban road model one ca either syntheize the parameters from the spreadsheet running
```
>> chania = SFMSynthesis('ChaniaUrbanRoadModel');
```
or load the MAT-file running
```
>> chania = load('ChaniaUrbanRoadModel/data.mat');
```
Note that using <tt>load</tt> it is possible to select only a fraction of the struct fields, *i.e.*
```
>> chania = load('ChaniaUrbanRoadModel/data.mat','junctions','links');
```
Thus, to implement a signal control startegy one one has to
- Set the directory of the model traffic network
- Sett the initial conditions and demand
- Synthesize the control policy
- Implement the control policy, *i.e.*, compute the green-times of the stages as a function of the link occupancy

### Example

***
  
## ✨ Contributing to SAFFRON

***

## 📄 License
[MIT License](https://github.com/decenter2021/SAFFRON/LICENSE)

***

## 💥 References 
<p align="justify">
<a href="https://doi.org/10.1016/j.trc.2008.10.002">Aboudolas, K., Papageorgiou, M. and Kosmatopoulos, E., 2009. Store-and-forward based methods for the signal control problem in large-scale congested urban road networks. Transportation Research Part C: Emerging Technologies, 17(2), pp.163-174. doi:10.1016/j.trc.2008.10.002.</a>
  
<a href="https://www.researchgate.net/profile/Christina-Diakaki/publication/270751666_Integrated_Control_of_Traffic_Flow_in_Corridor_Networks/links/569902fc08ae748dfaff351f/Integrated-Control-of-Traffic-Flow-in-Corridor-Networks.pdf">Diakaki, C., 1999. Integrated control of traffic flow in corridor networks. Ph. D. Thesis.</a>
 
<a href="https://doi.org/10.1007/BF01588328">Helgason, R., Kennington, J. and Lall, H., 1980. A polynomially bounded algorithm for a singly constrained quadratic program. Mathematical Programming, 18(1), pp.338-343. doi:10.1007/BF01588328.</a>
  
<a href="https://doi.org/10.1016/j.trc.2021.103412">Pedroso, L. and Batista, P., 2021. Decentralized store-and-forward based strategies for the signal control problem in large-scale congested urban road networks. Transportation Research Part C: Emerging Technologies, 132, p.103412. doi:10.1016/j.trc.2021.103412.</a>

<a href="">Pedroso, L., Batista, P., Papageorgiou, M. and Kosmatopoulos, E. [not published yet]</a>

</p>
