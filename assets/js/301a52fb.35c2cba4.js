"use strict";(self.webpackChunkstaticdocs_starter=self.webpackChunkstaticdocs_starter||[]).push([[601],{3905:(e,t,r)=>{r.r(t),r.d(t,{MDXContext:()=>l,MDXProvider:()=>p,mdx:()=>v,useMDXComponents:()=>m,withMDXComponents:()=>c});var a=r(67294);function n(e,t,r){return t in e?Object.defineProperty(e,t,{value:r,enumerable:!0,configurable:!0,writable:!0}):e[t]=r,e}function o(){return o=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var r=arguments[t];for(var a in r)Object.prototype.hasOwnProperty.call(r,a)&&(e[a]=r[a])}return e},o.apply(this,arguments)}function i(e,t){var r=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),r.push.apply(r,a)}return r}function s(e){for(var t=1;t<arguments.length;t++){var r=null!=arguments[t]?arguments[t]:{};t%2?i(Object(r),!0).forEach((function(t){n(e,t,r[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(r)):i(Object(r)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(r,t))}))}return e}function d(e,t){if(null==e)return{};var r,a,n=function(e,t){if(null==e)return{};var r,a,n={},o=Object.keys(e);for(a=0;a<o.length;a++)r=o[a],t.indexOf(r)>=0||(n[r]=e[r]);return n}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(a=0;a<o.length;a++)r=o[a],t.indexOf(r)>=0||Object.prototype.propertyIsEnumerable.call(e,r)&&(n[r]=e[r])}return n}var l=a.createContext({}),c=function(e){return function(t){var r=m(t.components);return a.createElement(e,o({},t,{components:r}))}},m=function(e){var t=a.useContext(l),r=t;return e&&(r="function"==typeof e?e(t):s(s({},t),e)),r},p=function(e){var t=m(e.components);return a.createElement(l.Provider,{value:t},e.children)},u="mdxType",f={inlineCode:"code",wrapper:function(e){var t=e.children;return a.createElement(a.Fragment,{},t)}},h=a.forwardRef((function(e,t){var r=e.components,n=e.mdxType,o=e.originalType,i=e.parentName,l=d(e,["components","mdxType","originalType","parentName"]),c=m(r),p=n,u=c["".concat(i,".").concat(p)]||c[p]||f[p]||o;return r?a.createElement(u,s(s({ref:t},l),{},{components:r})):a.createElement(u,s({ref:t},l))}));function v(e,t){var r=arguments,n=t&&t.mdxType;if("string"==typeof e||n){var o=r.length,i=new Array(o);i[0]=h;var s={};for(var d in t)hasOwnProperty.call(t,d)&&(s[d]=t[d]);s.originalType=e,s[u]="string"==typeof e?e:n,i[1]=s;for(var l=2;l<o;l++)i[l]=r[l];return a.createElement.apply(null,i)}return a.createElement.apply(null,r)}h.displayName="MDXCreateElement"},65654:(e,t,r)=>{r.r(t),r.d(t,{assets:()=>d,contentTitle:()=>i,default:()=>p,frontMatter:()=>o,metadata:()=>s,toc:()=>l});var a=r(87462),n=(r(67294),r(3905));const o={sidebar_position:7,id:"use-vrs",title:"Getting to Know and Use VRS"},i="Getting to Know and Use VRS Data",s={unversionedId:"use-vrs",id:"use-vrs",title:"Getting to Know and Use VRS",description:"Introduction",source:"@site/docs/use-vrs.mdx",sourceDirName:".",slug:"/use-vrs",permalink:"/Aria_data_tools/docs/use-vrs",draft:!1,editUrl:"https://github.com/facebookresearch/aria_data_tools/docs/use-vrs.mdx",tags:[],version:"current",sidebarPosition:7,frontMatter:{sidebar_position:7,id:"use-vrs",title:"Getting to Know and Use VRS"},sidebar:"tutorialSidebar",previous:{title:"How Project Aria Uses VRS",permalink:"/Aria_data_tools/docs/aria-vrs"},next:{title:"FAQ",permalink:"/Aria_data_tools/docs/faq"}},d={},l=[{value:"Introduction",id:"introduction",level:2},{value:"Instructions",id:"instructions",level:2},{value:"Check the VRS file\u2019s validity and integrity",id:"check-the-vrs-files-validity-and-integrity",level:2},{value:"Extract image or audio content to folders",id:"extract-image-or-audio-content-to-folders",level:2},{value:"Extract all content to folders and JSONs",id:"extract-all-content-to-folders-and-jsons",level:2},{value:"Inspect how many data recordings there are by type",id:"inspect-how-many-data-recordings-there-are-by-type",level:2}],c={toc:l},m="wrapper";function p(e){let{components:t,...r}=e;return(0,n.mdx)(m,(0,a.Z)({},c,r,{components:t,mdxType:"MDXLayout"}),(0,n.mdx)("h1",{id:"getting-to-know-and-use-vrs-data"},"Getting to Know and Use VRS Data"),(0,n.mdx)("h2",{id:"introduction"},"Introduction"),(0,n.mdx)("p",null,"Project Aria data is stored within ",(0,n.mdx)("a",{parentName:"p",href:"/Aria_data_tools/docs/aria-vrs"},"VRS")," data containers. This means that standard VRS commands can be used to extract and use the datasets. VRS is installed as part of Aria Research Kit: Aria Data Tools."),(0,n.mdx)("p",null,"You do not need to use these VRS commands, you can use the ",(0,n.mdx)("a",{parentName:"p",href:"/Aria_data_tools/docs/howto/dataprovider"},"Aria Data Provider")," in Aria Data Tools instead, but these options are available if you wish."),(0,n.mdx)("h2",{id:"instructions"},"Instructions"),(0,n.mdx)("p",null,"Use the following commands to examine, extract and inspect data contained in any VRS file."),(0,n.mdx)("h2",{id:"check-the-vrs-files-validity-and-integrity"},"Check the VRS file\u2019s validity and integrity"),(0,n.mdx)("p",null,"The ",(0,n.mdx)("inlineCode",{parentName:"p"},"check")," command decodes every record in the VRS file and prints how many records were decoded successfully. It proves that the VRS file is correct at the VRS level. You can also compute a checksum to ensure you have valid VRS files. For more information see ",(0,n.mdx)("a",{parentName:"p",href:"https://facebookresearch.github.io/vrs/docs/VrsCliTool#file-validation"},"VRS File Validation"),"."),(0,n.mdx)("pre",null,(0,n.mdx)("code",{parentName:"pre"},"$ vrs check <file.vrs>\n$ vrs checksum <file.vrs>\n")),(0,n.mdx)("p",null,"If the file is not valid, it\u2019s normally because there is missing data that could lead to invalid behavior with the tooling. All files in the ",(0,n.mdx)("a",{parentName:"p",href:"https://about.facebook.com/realitylabs/projectaria/datasets"},"Aria Pilot Dataset")," are valid, so if you encounter that issue with this dataset, re-downloading the file should resolve this issue."),(0,n.mdx)("h2",{id:"extract-image-or-audio-content-to-folders"},"Extract image or audio content to folders"),(0,n.mdx)("p",null,"Use the following commands to extract JPEG or WAV files.  Use the ",(0,n.mdx)("inlineCode",{parentName:"p"},"--to <folder_path>")," to specify a destination folder where the data will be extracted, or it will be added to the current working directory."),(0,n.mdx)("pre",null,(0,n.mdx)("code",{parentName:"pre"},"$ vrs extract-images <file.vrs> --to <imagefolder>\n$ vrs extract-audio <file.vrs> --to <audiofolder>\n")),(0,n.mdx)("p",null,"To extract RAW image files, use:"),(0,n.mdx)("pre",null,(0,n.mdx)("code",{parentName:"pre"},"vrs extract-images <file.vrs> --raw-images --to <imagefolder>\n")),(0,n.mdx)("h2",{id:"extract-all-content-to-folders-and-jsons"},"Extract all content to folders and JSONs"),(0,n.mdx)("p",null,"This command lets you extract all images, audio, and metadata into files:"),(0,n.mdx)("pre",null,(0,n.mdx)("code",{parentName:"pre"},"vrs extract-all <file.vrs> --to <folder>\n")),(0,n.mdx)("p",null,"The metadata is extracted into a single ",(0,n.mdx)("inlineCode",{parentName:"p"},".jsons")," file that contains a succession of json messages, one per line. Each line corresponds to a single record, in timestamp order, so it is possible to parse it even if the number of records is huge. Saving all the data in a single file prevents saturating your disk with possibly millions of small files."),(0,n.mdx)("p",null,"Once extracted, your file will look like this:"),(0,n.mdx)("pre",null,(0,n.mdx)("code",{parentName:"pre"},"    \u251c\u2500\u2500 file.vrs\n    \u251c\u2500\u2500 all_data\n      * `NNNN-MM` folders: image folders, one folder per stream containing images.\n      \u251c\u2500\u2500 1201-1 # SLAM Left images\n          \u251c\u2500\u2500 *.jpg\n      \u251c\u2500\u2500 1201-2 # SLAM Right images\n          \u251c\u2500\u2500 *.jpg\n      \u251c\u2500\u2500 211-1  # Eye Tracking images\n          \u251c\u2500\u2500 *.jpg\n      \u251c\u2500\u2500 214-1  # RGB (Color) Camera images\n          \u251c\u2500\u2500 *.jpg\n      \u251c\u2500\u2500 metadata.jsons\n      \u2514\u2500\u2500 ReadMe.md\n")),(0,n.mdx)("p",null,"For more information, see ",(0,n.mdx)("a",{parentName:"p",href:"https://facebookresearch.github.io/vrs/docs/VrsCliTool#data-extraction"},"VRS Data Extraction"),"."),(0,n.mdx)("h2",{id:"inspect-how-many-data-recordings-there-are-by-type"},"Inspect how many data recordings there are by type"),(0,n.mdx)("pre",null,(0,n.mdx)("code",{parentName:"pre"},'vrs <file.vrs> | grep "] records."\n')),(0,n.mdx)("p",null,"Will get you a return like this:"),(0,n.mdx)("p",null,"|Number of data recordings| Type of sensor | first or second data stream - device type [",(0,n.mdx)("a",{parentName:"p",href:"https://facebookresearch.github.io/vrs/docs/FileStructure"},"Stream ID number"),"]|"),(0,n.mdx)("pre",null,(0,n.mdx)("code",{parentName:"pre"},"623 Eye Camera Class #1 - device/aria [211-1] records.\n1244 RGB Camera Class #1 - device/aria [214-1] records.\n729 Stereo Audio Class #1 - device/aria [231-1] records.\n3101 Barometer Data Class #1 - device/aria [247-1] records.\n65 Time Domain Mapping Class #1 - device/aria [285-1] records.\n623 Camera Data (SLAM) #1 - device/aria [1201-1] records.\n623 Camera Data (SLAM) #2 - device/aria [1201-2] records.\n61965 IMU Data (SLAM) #1 - device/aria [1202-1] records.\n50002 IMU Data (SLAM) #2 - device/aria [1202-2] records.\n619 Magnetometer Data (SLAM) #1 - device/aria [1203-1] records.\n\n")))}p.isMDXComponent=!0}}]);