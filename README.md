# SAFFRON:<br><sub>Store-And-Forward model toolbox For urban ROad Network signal control in MATLAB</sub>

[![GitHub release](https://img.shields.io/github/release/Naereen/StrapDown.js.svg)](https://github.com/decenter2021/SAFFRON/releases)
[![MIT license](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/decenter2021/SAFFRON/LICENSE)
[![DOI:[not_published_yet]](https://zenodo.org/badge/DOI/not_published_yet.svg)](https://doi.org/not_published_yet.svg)

## Features
- Collection of tools for the **development** and **evaluation** of **store-and-forward** based strategies
- Full model of the urban traffic network of the city center of **Chania, Greece**
- Implementation of **state-of-the-art** traffic responsive signal control strategies



## Description
<p align="justify">
[link to article- description of store and forward there]
[Add brief description and motivation]
[say documentation below]
[probably add an index with links to the sections below]
</p>

## Authors 
Leonardo Pedroso<sup>1</sup> (<a href="https://github.com/leonardopedroso">GitHub</a>, <a href="https://scholar.google.com/citations?user=W7_Gq-0AAAAJ">Google Scholar</a>, <a href="https://orcid.org/0000-0002-1508-496X">ORCID</a>)<br>
Pedro Batista<sup>1</sup> (<a href="https://scholar.google.com/citations?user=6eon48IAAAAJ">Google Scholar</a>, <a href="https://orcid.org/0000-0001-6079-0436"> ORCID</a>)<br>
Markos Papageorgiou<sup>2</sup> (<a href="https://scholar.google.com/citations?user=sr8TOTQAAAAJ">Google Scholar</a>, <a href="https://orcid.org/0000-0001-5821-4982"> ORCID</a>)<br>
Elias Kosmatopoulos<sup>3</sup> (<a href="https://scholar.google.com/citations?user=LIRR8B8AAAAJ">Google Scholar</a>, <a href="https://orcid.org/0000-0002-3735-4238"> ORCID</a>)<br>
<sub>*<sup>1</sup>Institute for Systems and Robotics, Instituto Superior Técnico, Universidade de Lisboa, Portugal<br>
  <sup>2</sup>Dynamic Systems and Simulation Laboratory, Technical University of Crete, GR-73100 Chania, Greece<br>
  <sup>3</sup>Department of Electrical and Computer Engineering, Democritus University of Thrace, Xanthi, Greece<br>*</sub>

## Contact
SAFFRON toolbox is currently maintained by Leonardo Pedroso (<a href="mailto:leonardo.pedroso@tecnico.ulisboa.pt">leonardo.pedroso@tecnico.ulisboa.pt</a>).

## Documentation
### Synthesis
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
| <tt>Bu</tt> | State-space matrix ![$\mathbf{A}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B}) of the store-and-forward model in equation (4) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#references) | ![$\mathbf{B_u}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B_u})|
| <tt>BG</tt> | State-space matrix ![$\mathbf{A}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B}) of the store-and-forward model in equation (7) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#references) | ![$\mathbf{B_G}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B_G})|
| <tt>Bg</tt> | State-space matrix ![$\mathbf{A}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B}) of the store-and-forward model in equation (8) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#references) | ![$\mathbf{B_g}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B_g})|
| <tt>Bu_sim</tt> | State-space matrix ![$\mathbf{A}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B}) of the store-and-forward model in equation (4) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#references) discretized with simulation cycle ![$T$](https://render.githubusercontent.com/render/math?math=\color{gray}T) | - |
| <tt>BG_sim</tt> | State-space matrix ![$\mathbf{A}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B}) of the store-and-forward model in equation (7) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#references) discretized with simulation cycle ![$T$](https://render.githubusercontent.com/render/math?math=\color{gray}T) | - |
| <tt>Bg_sim</tt> | State-space matrix ![$\mathbf{A}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{B}) of the store-and-forward model in equation (8) of [(Pedroso, Batista, Papageorgiou, and Kosmatopoulos, 2022)](#references) discretized with simulation cycle ![$T$](https://render.githubusercontent.com/render/math?math=\color{gray}T) | - |
| <tt>C_z</tt> | State-space matrix ![$\mathbf{C}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{C}) of the store-and-forward model of the network | ![$\mathbf{C}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{C})|
| <tt>E_DTUC_psi</tt> | Sparsity matrix of the controller gain for the decentralized control strategy DTUC with decentralized configuration ![$\mathbf{\Psi}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{\Psi}) proposed in [(Pedroso and Batista, 2021)](#references) | ![$\mathbf{E_{\Psi}}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{E_{\Psi}})|
| <tt>E_DTUC_phi</tt> | Sparsity matrix of the controller gain for the decentralized control strategy DTUC with decentralized configuration ![$\mathbf{\Phi}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{\Phi}) proposed in [(Pedroso and Batista, 2021)](#references) | ![$\mathbf{E_{\Phi}}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{E_{\Phi}})|
| <tt>E_D2TUC_psi</tt> | Sparsity matrix of the controller gain for the decentralized control strategy D2TUC with decentralized configuration ![$\mathbf{\Psi}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{\Psi}) proposed in [(Pedroso and Batista, 2021)](#references) | ![$\mathbf{E_{\Psi}}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{E_{\Psi}})|
| <tt>E_D2TUC_phi</tt> | Sparsity matrix of the controller gain for the decentralized control strategy D2TUC with decentralized configuration ![$\mathbf{\Phi}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{\Phi}) proposed in [(Pedroso and Batista, 2021)](#references) | ![$\mathbf{E_{\Phi}}$](https://render.githubusercontent.com/render/math?math=\color{gray}\mathbf{E_{\Phi}})|

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

### Simulation script

### Chania urban road network

### Example
  
## Contributing to SAFFRON

## License
[MIT License](https://github.com/decenter2021/SAFFRON/LICENSE)

## References 
<p align="justify">
<a href="https://doi.org/10.1016/j.trc.2008.10.002">Aboudolas, K., Papageorgiou, M. and Kosmatopoulos, E., 2009. Store-and-forward based methods for the signal control problem in large-scale congested urban road networks. Transportation Research Part C: Emerging Technologies, 17(2), pp.163-174.</a>
  
<a href="https://www.researchgate.net/profile/Christina-Diakaki/publication/270751666_Integrated_Control_of_Traffic_Flow_in_Corridor_Networks/links/569902fc08ae748dfaff351f/Integrated-Control-of-Traffic-Flow-in-Corridor-Networks.pdf">Diakaki, C., 1999. Integrated control of traffic flow in corridor networks. Ph. D. Thesis.</a>
  
<a href="https://doi.org/10.1016/j.trc.2021.103412">Pedroso, L. and Batista, P., 2021. Decentralized store-and-forward based strategies for the signal control problem in large-scale congested urban road networks. Transportation Research Part C: Emerging Technologies, 132, p.103412.</a>
  
Pedroso, L., Batista, P., Papageorgiou, M. and Kosmatopoulos, E. [not published yet]
</p>
