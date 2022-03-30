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

| Field      | Description |
| ----------- | ----------- |
| a      | b       |
| b   | a        |



>Example: *Load the Chania urban road network provided in SAFFRON*
>```
>>> chania = SFMSynthesis('ChaniaUrbanRoadModel')
>>> chania.S
> ans = 
>     42

  
  
## Contributing to SAFFRON

## License
[MIT License](https://github.com/decenter2021/SAFFRON/LICENSE)

## References 
<p align="justify">
Aboudolas, K., Papageorgiou, M. and Kosmatopoulos, E., 2009. Store-and-forward based methods for the signal control problem in large-scale congested urban road networks. Transportation Research Part C: Emerging Technologies, 17(2), pp.163-174.
  
Diakaki, C., 1999. Integrated control of traffic flow in corridor networks. Ph. D. Thesis.
  
Pedroso, L. and Batista, P., 2021. Decentralized store-and-forward based strategies for the signal control problem in large-scale congested urban road networks. Transportation Research Part C: Emerging Technologies, 132, p.103412.
  
Pedroso, L., Batista, P., Aboudolas, K. and Kosmatopoulos, E. [not published yet]
</p>
