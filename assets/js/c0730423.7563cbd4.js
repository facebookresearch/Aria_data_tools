"use strict";(self.webpackChunkstaticdocs_starter=self.webpackChunkstaticdocs_starter||[]).push([[157],{3905:(e,t,n)=>{n.r(t),n.d(t,{MDXContext:()=>m,MDXProvider:()=>u,mdx:()=>f,useMDXComponents:()=>c,withMDXComponents:()=>d});var a=n(67294);function r(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function i(){return i=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var n=arguments[t];for(var a in n)Object.prototype.hasOwnProperty.call(n,a)&&(e[a]=n[a])}return e},i.apply(this,arguments)}function l(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,a)}return n}function o(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?l(Object(n),!0).forEach((function(t){r(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):l(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function p(e,t){if(null==e)return{};var n,a,r=function(e,t){if(null==e)return{};var n,a,r={},i=Object.keys(e);for(a=0;a<i.length;a++)n=i[a],t.indexOf(n)>=0||(r[n]=e[n]);return r}(e,t);if(Object.getOwnPropertySymbols){var i=Object.getOwnPropertySymbols(e);for(a=0;a<i.length;a++)n=i[a],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(r[n]=e[n])}return r}var m=a.createContext({}),d=function(e){return function(t){var n=c(t.components);return a.createElement(e,i({},t,{components:n}))}},c=function(e){var t=a.useContext(m),n=t;return e&&(n="function"==typeof e?e(t):o(o({},t),e)),n},u=function(e){var t=c(e.components);return a.createElement(m.Provider,{value:t},e.children)},s="mdxType",x={inlineCode:"code",wrapper:function(e){var t=e.children;return a.createElement(a.Fragment,{},t)}},h=a.forwardRef((function(e,t){var n=e.components,r=e.mdxType,i=e.originalType,l=e.parentName,m=p(e,["components","mdxType","originalType","parentName"]),d=c(n),u=r,s=d["".concat(l,".").concat(u)]||d[u]||x[u]||i;return n?a.createElement(s,o(o({ref:t},m),{},{components:n})):a.createElement(s,o({ref:t},m))}));function f(e,t){var n=arguments,r=t&&t.mdxType;if("string"==typeof e||r){var i=n.length,l=new Array(i);l[0]=h;var o={};for(var p in t)hasOwnProperty.call(t,p)&&(o[p]=t[p]);o.originalType=e,o[s]="string"==typeof e?e:r,l[1]=o;for(var m=2;m<i;m++)l[m]=n[m];return a.createElement.apply(null,l)}return a.createElement.apply(null,n)}h.displayName="MDXCreateElement"},45531:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>p,contentTitle:()=>l,default:()=>u,frontMatter:()=>i,metadata:()=>o,toc:()=>m});var a=n(87462),r=(n(67294),n(3905));const i={sidebar_position:5,id:"speech2text",title:"Speech2Text Output Data"},l="Speech2Text Output Data",o={unversionedId:"pilotdata/speech2text",id:"pilotdata/speech2text",title:"Speech2Text Output Data",description:"Speech2Text Output Data provides text strings generated by Automatic Speech Recognition with timestamps and confidence rating.",source:"@site/docs/pilotdata/speech2text.mdx",sourceDirName:"pilotdata",slug:"/pilotdata/speech2text",permalink:"/Aria_data_tools/docs/pilotdata/speech2text",draft:!1,editUrl:"https://github.com/facebookresearch/aria_data_tools/docs/pilotdata/speech2text.mdx",tags:[],version:"current",sidebarPosition:5,frontMatter:{sidebar_position:5,id:"speech2text",title:"Speech2Text Output Data"},sidebar:"tutorialSidebar",previous:{title:"Location Output Data",permalink:"/Aria_data_tools/docs/pilotdata/location-output"},next:{title:"Eye Gaze Data",permalink:"/Aria_data_tools/docs/pilotdata/reprojected-gaze"}},p={},m=[],d={toc:m},c="wrapper";function u(e){let{components:t,...n}=e;return(0,r.mdx)(c,(0,a.Z)({},d,n,{components:t,mdxType:"MDXLayout"}),(0,r.mdx)("h1",{id:"speech2text-output-data"},"Speech2Text Output Data"),(0,r.mdx)("p",null,"Speech2Text Output Data provides text strings generated by Automatic Speech Recognition with timestamps and confidence rating."),(0,r.mdx)("p",null,"Each recording has two .csv files that are the same, except  ",(0,r.mdx)("inlineCode",{parentName:"p"},"speech2text/speech.csv")," uses the wav file time domain and  ",(0,r.mdx)("inlineCode",{parentName:"p"},"speech2text/speech_aria_domain.csv")," uses Aria time domain."),(0,r.mdx)("p",null,(0,r.mdx)("strong",{parentName:"p"},"Table 1:")," ",(0,r.mdx)("em",{parentName:"p"},(0,r.mdx)("inlineCode",{parentName:"em"},"speech.csv")," Structure")),(0,r.mdx)("table",null,(0,r.mdx)("thead",{parentName:"table"},(0,r.mdx)("tr",{parentName:"thead"},(0,r.mdx)("th",{parentName:"tr",align:null},"startTime_ms"),(0,r.mdx)("th",{parentName:"tr",align:null},"endTime_ms"),(0,r.mdx)("th",{parentName:"tr",align:null},"written"),(0,r.mdx)("th",{parentName:"tr",align:null},"confidence"))),(0,r.mdx)("tbody",{parentName:"table"},(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"54040"),(0,r.mdx)("td",{parentName:"tr",align:null},"55040"),(0,r.mdx)("td",{parentName:"tr",align:null},"I\u2019m"),(0,r.mdx)("td",{parentName:"tr",align:null},"0.25608")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"72920"),(0,r.mdx)("td",{parentName:"tr",align:null},"73920"),(0,r.mdx)("td",{parentName:"tr",align:null},"looking"),(0,r.mdx)("td",{parentName:"tr",align:null},"0.84339")))),(0,r.mdx)("p",null,(0,r.mdx)("em",{parentName:"p"},"Note:")," token in wav file time domain (start = 0)"),(0,r.mdx)("p",null,(0,r.mdx)("strong",{parentName:"p"},"Table 2:")," ",(0,r.mdx)("em",{parentName:"p"},(0,r.mdx)("inlineCode",{parentName:"em"},"speech_aria_domain.csv")," Structure")),(0,r.mdx)("table",null,(0,r.mdx)("thead",{parentName:"table"},(0,r.mdx)("tr",{parentName:"thead"},(0,r.mdx)("th",{parentName:"tr",align:null},"startTime_ns"),(0,r.mdx)("th",{parentName:"tr",align:null},"endTime_ns"),(0,r.mdx)("th",{parentName:"tr",align:null},"written"),(0,r.mdx)("th",{parentName:"tr",align:null},"confidence"))),(0,r.mdx)("tbody",{parentName:"table"},(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"56511040"),(0,r.mdx)("td",{parentName:"tr",align:null},"56512040"),(0,r.mdx)("td",{parentName:"tr",align:null},"I\u2019m"),(0,r.mdx)("td",{parentName:"tr",align:null},"0.25608")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"56529920"),(0,r.mdx)("td",{parentName:"tr",align:null},"56530920"),(0,r.mdx)("td",{parentName:"tr",align:null},"looking"),(0,r.mdx)("td",{parentName:"tr",align:null},"0.84339")))),(0,r.mdx)("p",null,(0,r.mdx)("em",{parentName:"p"},"Note:")," token in Aria file time domain (start = 0)"))}u.isMDXComponent=!0}}]);