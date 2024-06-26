# SAFFRON:<br><sub>🚦Store-And-Forward model toolbox For urban ROad Network signal control in MATLAB</sub>

[![GitHub release](https://img.shields.io/github/release/Naereen/StrapDown.js.svg)](https://github.com/decenter2021/SAFFRON/releases)
[![MIT license](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/decenter2021/SAFFRON/LICENSE)
[![DOI:[not_published_yet]](https://zenodo.org/badge/DOI/10.1109/ITSC55140.2022.9922508.svg)](https://doi.org/10.1109/ITSC55140.2022.9922508.svg)

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

<p align="justify">
<a href="https://ieeexplore.ieee.org/abstract/document/9922508">Pedroso, L., Batista, P., Papageorgiou, M. and Kosmatopoulos, E., 2022. SAFFRON: Store-And-Forward model toolbox For urban ROad Network signal control in MATLAB. 2022 IEEE 25th International Conference on Intelligent Transportation Systems (ITSC), pp. 3698-3703. doi:10.1109/ITSC55140.2022.9922508.</a>
</p>

[[Accepted version]](http://web.tecnico.ulisboa.pt/ist189691/publications/2022_11_pedroso_saffron_itsc_goa.pdf)


If you use SAFFRON, reference the publication above.

~~~BibTeX
@inproceedings{PedrosoBatistaEtAl2022Saffron,
	author = {Leonardo Pedroso and Pedro Batista and Markos Papageorgiou and Elias Kosmatopoulos},
	title = {{SAFFRON}: {Store-And-Forward} model toolbox {For} urban {ROad} {Network} signal control in {MATLAB}},
	booktitle = {2022 IEEE 25th International Conference on Intelligent Transportation Systems (ITSC)},
	year = {2022},
	pages = {3698-3703},
	doi = {10.1109/ITSC55140.2022.9922508}
}
~~~

SAFFRON allows to:
- Seamlessly simulate store-and-forward based signal control strategies
- Apply novel solutions to a meaningful model that is publicly available and, hence, can be reproduced
- Compare novel solutions with other control strategies with little effort

Low-level thorough [documentation](#-documentation) is provided in the repository page, as well as in the source files.

The community is encouraged to [contribute](#-contributing-to-saffron) to SAFFRON with suggestions, additions, and the implementation of signal control strategies.

***

## ✍🏼 Authors 
Leonardo Pedroso<sup>1</sup> <a href="https://scholar.google.com/citations?user=W7_Gq-0AAAAJ"><img src="https://cdn.icon-icons.com/icons2/2108/PNG/512/google_scholar_icon_130918.png" style="width:1em;margin-right:.5em;"></a> <a href="https://orcid.org/0000-0002-1508-496X"><img src="https://orcid.org/sites/default/files/images/orcid_16x16.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a> <a href="https://github.com/leonardopedroso"><img src="https://github.githubassets.com/images/modules/logos_page/GitHub-Mark.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a><br>
Pedro Batista<sup>1</sup> <a href="https://scholar.google.com/citations?user=6eon48IAAAAJ"><img src="https://cdn.icon-icons.com/icons2/2108/PNG/512/google_scholar_icon_130918.png" style="width:1em;margin-right:.5em;"></a> <a href="https://orcid.org/0000-0001-6079-0436"><img src="https://orcid.org/sites/default/files/images/orcid_16x16.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a><br>
Markos Papageorgiou<sup>2,3</sup> <a href="https://scholar.google.com/citations?user=sr8TOTQAAAAJ"><img src="https://cdn.icon-icons.com/icons2/2108/PNG/512/google_scholar_icon_130918.png" style="width:1em;margin-right:.5em;"></a> <a href="https://orcid.org/0000-0001-5821-4982"><img src="https://orcid.org/sites/default/files/images/orcid_16x16.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a><br>
Elias Kosmatopoulos<sup>4</sup> <a href="https://scholar.google.com/citations?user=LIRR8B8AAAAJ"><img src="https://cdn.icon-icons.com/icons2/2108/PNG/512/google_scholar_icon_130918.png" style="width:1em;margin-right:.5em;"></a> <a href="https://orcid.org/0000-0002-3735-4238"><img src="https://orcid.org/sites/default/files/images/orcid_16x16.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a><br>
<sub>*<sup>1</sup>Institute for Systems and Robotics, Instituto Superior Técnico, Universidade de Lisboa, Portugal<br>
  <sup>2</sup>Dynamic Systems and Simulation Laboratory, Technical University of Crete, Chania, Greece<br>
  <sup>3</sup>Faculty of Maritime and Transportation, Ningbo University, Ningbo, China<br>
  <sup>4</sup>Department of Electrical and Computer Engineering, Democritus University of Thrace, Xanthi, Greece<br>*</sub>





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

- the number of junctions $J$, links $Z$, and stages $S$, control cycle $C$, simulation cycle $T$, and the upstream gating parameter $c_ug$
- the lost time and number of stages in each junction 
- the capacity, saturation flow, number of lanes, initial number of vehicles, and demand flow for each link
- the minimum green time and historic green time of each stage
- the stage matrix $\mathbf{S}$, *i.e.*, a table that indicates which links have right of way (r.o.w.) for each stage
- the turning rates matrix $\mathbf{T}$, *i.e.*, a table that indicates the probability of turning into the links of the network on the exit of a certain link, and the exit rate of all links $\mathbf{t_0}$

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
| <tt>J</tt> | Number of junctions | $J$|
| <tt>Z</tt> | Number of junctions | $L$|
| <tt>nStages</tt> | Number of stages | $L$|
| <tt>C</tt> | Control cycle (s) | $C$|
| <tt>c</tt> | Upstream gating parameter | $c_{ug}$|
| <tt>Tsim</tt> | Simulation cycle (s) | $T$|
| <tt>lostTime</tt> | Column vector of lost times (s) in each junction  | $\mathrm{col}(L_1,\ldots,L_J)$|
| <tt>nStagesJunction</tt> | Column vector of number of stages of each junction | $\mathrm{col}(&#124\mathcal{F}_1&#124,\ldots,&#124\mathcal{F}_S&#124))$|
| <tt>capacity</tt> | Column vector of capacity of each link (veh) | $\mathrm{col}(x_{1,\text{max}},\ldots,x_{Z,\text{max}})$|
| <tt>saturation</tt> | Column vector of saturation flow of each link (veh/s) | $\mathrm{col}(S_1,\ldots,S_Z)$|
| <tt>lanes</tt> | Column vector of number of lanes in each link | - |
| <tt>x0</tt> | Initial number of vehicles of each link (veh) | $\mathbf{x}(0)$|
| <tt>d</tt> | Column vector of demand on each link (veh/s) | $\mathrm{col}(d_1,\ldots,d_Z)$|
| <tt>gmin</tt> | Column vector of minimum green time of each stage (s) | $\mathrm{col}(g_{1,min},\ldots,g_{S,min})$|
| <tt>gN</tt> |  Column vector of historic green time of each stage (s) | $\mathbf{g_N}$|
| <tt>T</tt> | Turning rates matrix | $\mathbf{T}$|
| <tt>t0</tt> | Exit rates vector | $\mathbf{t_0}$|
| <tt>S</tt> | Stage matrix | $\mathbf{S}$|
| <tt>junctions</tt> | Cell array indexed by junction number that contains the number of the stages associated with that junction (see example below) | -|
| <tt>links</tt> | $Z\times 2$ matrix whose rows are indexed by link number and the corresponding columns are the origin and destination junctions respectively, *i.e.*, it is the ordered list of edges of the network graph (see example below) | - |
| <tt>inLinks</tt> | Column vector of link indices that originate from outside the network | - |
| <tt>notInLinks</tt> | Column vector of link indices that do not originate from outside the network | - |
| <tt>A</tt> | State-space matrix $\mathbf{A}$ of the store-and-forward model of the network | $\mathbf{A}$|
| <tt>Bu</tt> | State-space matrix $\mathbf{A}$ of the store-and-forward model in equation (4) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#-references) | $\mathbf{B_u}$|
| <tt>BG</tt> | State-space matrix $\mathbf{A}$ of the store-and-forward model in equation (7) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#-references) | $\mathbf{B_G}$|
| <tt>Bg</tt> | State-space matrix $\mathbf{A}$ of the store-and-forward model in equation (8) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#-references) | $\mathbf{B_g}$|
| <tt>Bu_sim</tt> | State-space matrix $\mathbf{A}$ of the store-and-forward model in equation (4) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#-references) discretized with simulation cycle $T$ | - |
| <tt>BG_sim</tt> | State-space matrix $\mathbf{A}$ of the store-and-forward model in equation (7) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#-references) discretized with simulation cycle $T$ | - |
| <tt>Bg_sim</tt> | State-space matrix $\mathbf{A}$ of the store-and-forward model in equation (8) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#-references) discretized with simulation cycle $T$ | - |
| <tt>C_z</tt> | State-space matrix $\mathbf{C}$ of the store-and-forward model of the network | $\mathbf{C}$|
| <tt>E_DTUC_psi</tt> | Sparsity matrix of the controller gain for the decentralized control strategy DTUC with decentralized configuration $\mathbf{\Psi}$ proposed in [(Pedroso and Batista, 2021)](#-references) | $\mathbf{E_{\Psi}}$|
| <tt>E_DTUC_phi</tt> | Sparsity matrix of the controller gain for the decentralized control strategy DTUC with decentralized configuration $\mathbf{\Phi}$ proposed in [(Pedroso and Batista, 2021)](#-references) | $\mathbf{E_{\Phi}}$|
| <tt>E_D2TUC_psi</tt> | Sparsity matrix of the controller gain for the decentralized control strategy D2TUC with decentralized configuration $\mathbf{\Psi}$ proposed in [(Pedroso and Batista, 2021)](#-references) | $\mathbf{E_{\Psi}}$|
| <tt>E_D2TUC_phi</tt> | Sparsity matrix of the controller gain for the decentralized control strategy D2TUC with decentralized configuration $\mathbf{\Phi}$ proposed in [(Pedroso and Batista, 2021)](#-references) | $\mathbf{E_{\Phi}}$|

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
$\mathrm{TTS} = T \sum\limits_{k}^{} \sum\limits_{z=1}^Z x_z(k)$
</p><br>
and relative queue balance (RQB) <br>
<p align="center">
$\mathrm{RQB} = \sum\limits_{k}^{} \sum\limits_{z=1}^Z \frac{x_z^2(k)}{x_{z,max}}$
</p><br>
<p align="justify">
introcuced in <a href="#-references">(Aboudolas, Papageorgiou, and Kosmatopoulos, 2009)</a> can be computed seamlessly with SAFFRON. 
Let <tt>xNL</tt> be a $Z\times N_{sim}$ array, where $N_{sim}$ is the number of cycles that were simulated of a model <tt>model</tt> with the nonlinear dynamiccs (See <a href="simulation-script">Simulation script</a> for a template to simulate the nonlinear dynamics and <a href="example">Example</a> for an example). The $k$-th row of <tt>xNL</tt>, <i>i.e.</i> <tt>xNL(:,k)</tt>, is the column vector of the link occupancy at time $t = (k-1)T$. The command
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

A control policy can be implemented seamlessly in this script by 
- Setting the directory of the traffic network model
- Setting the initial occupancy and demand
- Synthesizing the novel control policy
- Implementing the novel control policy, *i.e.*, compute the green-times of the stages as a function of the link occupancy

in the places indicated in the script. The template file <tt>simulation_template.m</tt> is well commented so that it is very easy to adapt the script.

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

### Example

Full source code of the application of SAFFRON to the implementation of

- the well-known TUC strategy <a href="#-references">(Diakaki, 1999)</a>
- two recent decentralized signal control strategies proposed in <a href="#-references">(Pedroso and Batista, 2021)</a>

is provided in the folder <tt>Examples/PedrosoBatista2021</tt>.

<p align="justify">
To run this example place the simulation script and auxiliary <tt>.m</tt> files in <tt>Examples/PedrosoBatista2021</tt> in a directory with the SAFFRON source files. Alternatively, place the toolbox source files in a directory and <a href="https://www.mathworks.com/help/matlab/ref/addpath.html">add it to the MATLAB search path</a>.
</p>

>Example: *Run simulation example of <a href="#-references">(Pedroso and Batista, 2021)</a>*
>```
>>> simulation

<p align="justify">
The simulation of the the D2TUC strategy with decentralized configuration $\mathbf{\Phi}$  using SAFFRON tools in the Chania urban road network that is provided has the following occupancy and green time evolution
</p>

![occupancy](https://user-images.githubusercontent.com/40807922/161295642-a5bc964e-345a-469a-9694-d7671a09639c.png)
![green_times](https://user-images.githubusercontent.com/40807922/161295624-410759be-aefb-4fec-8f33-abc36e766092.png)
  
***
  
## ✨ Contributing to SAFFRON

The community is encouraged to contribute with 
- Suggestions
- Addition of tools
- Implementations of signal control strategies

To contribute to SAFFRON 

- Open an issue ([tutorial on how to create an issue](https://docs.github.com/en/issues/tracking-your-work-with-issues/creating-an-issue))
- Make a pull request ([tutorial on how to contribute to GitHub projects](https://docs.github.com/en/get-started/quickstart/contributing-to-projects))
- Or, if you are not familiar with GitHub, [contact the authors](#-contact) 

***

## 📄 License
[MIT License](https://github.com/decenter2021/SAFFRON/blob/master/LICENSE)

***

## 💥 References 
<p align="justify">

<a href="https://ieeexplore.ieee.org/abstract/document/9922508">Pedroso, L., Batista, P., Papageorgiou, M. and Kosmatopoulos, E., 2022. SAFFRON: Store-And-Forward model toolbox For urban ROad Network signal control in MATLAB. 2022 IEEE 25th International Conference on Intelligent Transportation Systems (ITSC), pp. 3698-3703. doi:10.1109/ITSC55140.2022.9922508.</a>
  
<a href="https://doi.org/10.1016/j.trc.2008.10.002">Aboudolas, K., Papageorgiou, M. and Kosmatopoulos, E., 2009. Store-and-forward based methods for the signal control problem in large-scale congested urban road networks. Transportation Research Part C: Emerging Technologies, 17(2), pp.163-174. doi:10.1016/j.trc.2008.10.002.</a>
  
<a href="https://www.researchgate.net/profile/Christina-Diakaki/publication/270751666_Integrated_Control_of_Traffic_Flow_in_Corridor_Networks/links/569902fc08ae748dfaff351f/Integrated-Control-of-Traffic-Flow-in-Corridor-Networks.pdf">Diakaki, C., 1999. Integrated control of traffic flow in corridor networks. Ph. D. Thesis.</a>
 
<a href="https://doi.org/10.1007/BF01588328">Helgason, R., Kennington, J. and Lall, H., 1980. A polynomially bounded algorithm for a singly constrained quadratic program. Mathematical Programming, 18(1), pp.338-343. doi:10.1007/BF01588328.</a>
  
<a href="https://doi.org/10.1016/j.trc.2021.103412">Pedroso, L. and Batista, P., 2021. Decentralized store-and-forward based strategies for the signal control problem in large-scale congested urban road networks. Transportation Research Part C: Emerging Technologies, 132, p.103412. doi:10.1016/j.trc.2021.103412.</a>



</p>
