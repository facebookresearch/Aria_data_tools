"use strict";(self.webpackChunkstaticdocs_starter=self.webpackChunkstaticdocs_starter||[]).push([[514,972],{19963:(e,t,a)=>{a.r(t),a.d(t,{default:()=>pe});var n=a(67294),l=a(86010),o=a(1944),r=a(35281),i=a(43320),c=a(53438),s=a(74477),d=a(1116),m=a(39483),u=a(95999),b=a(12466),p=a(85936);const h={backToTopButton:"backToTopButton_sjWU",backToTopButtonShow:"backToTopButtonShow_xfvO"};function E(){const{shown:e,scrollToTop:t}=function(e){let{threshold:t}=e;const[a,l]=(0,n.useState)(!1),o=(0,n.useRef)(!1),{startScroll:r,cancelScroll:i}=(0,b.Ct)();return(0,b.RF)(((e,a)=>{let{scrollY:n}=e;const r=a?.scrollY;r&&(o.current?o.current=!1:n>=r?(i(),l(!1)):n<t?l(!1):n+window.innerHeight<document.documentElement.scrollHeight&&l(!0))})),(0,p.S)((e=>{e.location.hash&&(o.current=!0,l(!1))})),{shown:a,scrollToTop:()=>r(0)}}({threshold:300});return n.createElement("button",{"aria-label":(0,u.translate)({id:"theme.BackToTopButton.buttonAriaLabel",message:"Scroll back to top",description:"The ARIA label for the back to top button"}),className:(0,l.default)("clean-btn",r.k.common.backToTopButton,h.backToTopButton,e&&h.backToTopButtonShow),type:"button",onClick:t})}var f=a(16550),g=a(87524),v=a(86668),k=a(21327),_=a(87462);function C(e){return n.createElement("svg",(0,_.Z)({width:"20",height:"20","aria-hidden":"true"},e),n.createElement("g",{fill:"#7a7a7a"},n.createElement("path",{d:"M9.992 10.023c0 .2-.062.399-.172.547l-4.996 7.492a.982.982 0 01-.828.454H1c-.55 0-1-.453-1-1 0-.2.059-.403.168-.551l4.629-6.942L.168 3.078A.939.939 0 010 2.528c0-.548.45-.997 1-.997h2.996c.352 0 .649.18.828.45L9.82 9.472c.11.148.172.347.172.55zm0 0"}),n.createElement("path",{d:"M19.98 10.023c0 .2-.058.399-.168.547l-4.996 7.492a.987.987 0 01-.828.454h-3c-.547 0-.996-.453-.996-1 0-.2.059-.403.168-.551l4.625-6.942-4.625-6.945a.939.939 0 01-.168-.55 1 1 0 01.996-.997h3c.348 0 .649.18.828.45l4.996 7.492c.11.148.168.347.168.55zm0 0"})))}const S={collapseSidebarButton:"collapseSidebarButton_PEFL",collapseSidebarButtonIcon:"collapseSidebarButtonIcon_kv0_"};function N(e){let{onClick:t}=e;return n.createElement("button",{type:"button",title:(0,u.translate)({id:"theme.docs.sidebar.collapseButtonTitle",message:"Collapse sidebar",description:"The title attribute for collapse button of doc sidebar"}),"aria-label":(0,u.translate)({id:"theme.docs.sidebar.collapseButtonAriaLabel",message:"Collapse sidebar",description:"The title attribute for collapse button of doc sidebar"}),className:(0,l.default)("button button--secondary button--outline",S.collapseSidebarButton),onClick:t},n.createElement(C,{className:S.collapseSidebarButtonIcon}))}var I=a(59689),T=a(902);const x=Symbol("EmptyContext"),B=n.createContext(x);function y(e){let{children:t}=e;const[a,l]=(0,n.useState)(null),o=(0,n.useMemo)((()=>({expandedItem:a,setExpandedItem:l})),[a]);return n.createElement(B.Provider,{value:o},t)}var w=a(86043),L=a(48596),A=a(39960),H=a(72389);function M(e){let{categoryLabel:t,onClick:a}=e;return n.createElement("button",{"aria-label":(0,u.translate)({id:"theme.DocSidebarItem.toggleCollapsedCategoryAriaLabel",message:"Toggle the collapsible sidebar category '{label}'",description:"The ARIA label to toggle the collapsible sidebar category"},{label:t}),type:"button",className:"clean-btn menu__caret",onClick:a})}function F(e){let{item:t,onItemClick:a,activePath:o,level:i,index:s,...d}=e;const{items:m,label:u,collapsible:b,className:p,href:h}=t,{docs:{sidebar:{autoCollapseCategories:E}}}=(0,v.L)(),f=function(e){const t=(0,H.default)();return(0,n.useMemo)((()=>e.href?e.href:!t&&e.collapsible?(0,c.Wl)(e):void 0),[e,t])}(t),g=(0,c._F)(t,o),k=(0,L.Mg)(h,o),{collapsed:C,setCollapsed:S}=(0,w.u)({initialState:()=>!!b&&(!g&&t.collapsed)}),{expandedItem:N,setExpandedItem:I}=function(){const e=(0,n.useContext)(B);if(e===x)throw new T.i6("DocSidebarItemsExpandedStateProvider");return e}(),y=function(e){void 0===e&&(e=!C),I(e?null:s),S(e)};return function(e){let{isActive:t,collapsed:a,updateCollapsed:l}=e;const o=(0,T.D9)(t);(0,n.useEffect)((()=>{t&&!o&&a&&l(!1)}),[t,o,a,l])}({isActive:g,collapsed:C,updateCollapsed:y}),(0,n.useEffect)((()=>{b&&null!=N&&N!==s&&E&&S(!0)}),[b,N,s,S,E]),n.createElement("li",{className:(0,l.default)(r.k.docs.docSidebarItemCategory,r.k.docs.docSidebarItemCategoryLevel(i),"menu__list-item",{"menu__list-item--collapsed":C},p)},n.createElement("div",{className:(0,l.default)("menu__list-item-collapsible",{"menu__list-item-collapsible--active":k})},n.createElement(A.default,(0,_.Z)({className:(0,l.default)("menu__link",{"menu__link--sublist":b,"menu__link--sublist-caret":!h&&b,"menu__link--active":g}),onClick:b?e=>{a?.(t),h?y(!1):(e.preventDefault(),y())}:()=>{a?.(t)},"aria-current":k?"page":void 0,"aria-expanded":b?!C:void 0,href:b?f??"#":f},d),u),h&&b&&n.createElement(M,{categoryLabel:u,onClick:e=>{e.preventDefault(),y()}})),n.createElement(w.z,{lazy:!0,as:"ul",className:"menu__list",collapsed:C},n.createElement(K,{items:m,tabIndex:C?-1:0,onItemClick:a,activePath:o,level:i+1})))}var P=a(13919),W=a(39471);const Z={menuExternalLink:"menuExternalLink_NmtK"};function D(e){let{item:t,onItemClick:a,activePath:o,level:i,index:s,...d}=e;const{href:m,label:u,className:b,autoAddBaseUrl:p}=t,h=(0,c._F)(t,o),E=(0,P.Z)(m);return n.createElement("li",{className:(0,l.default)(r.k.docs.docSidebarItemLink,r.k.docs.docSidebarItemLinkLevel(i),"menu__list-item",b),key:u},n.createElement(A.default,(0,_.Z)({className:(0,l.default)("menu__link",!E&&Z.menuExternalLink,{"menu__link--active":h}),autoAddBaseUrl:p,"aria-current":h?"page":void 0,to:m},E&&{onClick:a?()=>a(t):void 0},d),u,!E&&n.createElement(W.Z,null)))}const R={menuHtmlItem:"menuHtmlItem_M9Kj"};function V(e){let{item:t,level:a,index:o}=e;const{value:i,defaultStyle:c,className:s}=t;return n.createElement("li",{className:(0,l.default)(r.k.docs.docSidebarItemLink,r.k.docs.docSidebarItemLinkLevel(a),c&&[R.menuHtmlItem,"menu__list-item"],s),key:o,dangerouslySetInnerHTML:{__html:i}})}function z(e){let{item:t,...a}=e;switch(t.type){case"category":return n.createElement(F,(0,_.Z)({item:t},a));case"html":return n.createElement(V,(0,_.Z)({item:t},a));default:return n.createElement(D,(0,_.Z)({item:t},a))}}function U(e){let{items:t,...a}=e;return n.createElement(y,null,t.map(((e,t)=>n.createElement(z,(0,_.Z)({key:t,item:e,index:t},a)))))}const K=(0,n.memo)(U),j={menu:"menu_SIkG",menuWithAnnouncementBar:"menuWithAnnouncementBar_GW3s"};function q(e){let{path:t,sidebar:a,className:o}=e;const i=function(){const{isActive:e}=(0,I.nT)(),[t,a]=(0,n.useState)(e);return(0,b.RF)((t=>{let{scrollY:n}=t;e&&a(0===n)}),[e]),e&&t}();return n.createElement("nav",{"aria-label":(0,u.translate)({id:"theme.docs.sidebar.navAriaLabel",message:"Docs sidebar",description:"The ARIA label for the sidebar navigation"}),className:(0,l.default)("menu thin-scrollbar",j.menu,i&&j.menuWithAnnouncementBar,o)},n.createElement("ul",{className:(0,l.default)(r.k.docs.docSidebarMenu,"menu__list")},n.createElement(K,{items:a,activePath:t,level:1})))}const G={sidebar:"sidebar_njMd",sidebarWithHideableNavbar:"sidebarWithHideableNavbar_wUlq",sidebarHidden:"sidebarHidden_VK0M",sidebarLogo:"sidebarLogo_isFc"};function Y(e){let{path:t,sidebar:a,onCollapse:o,isHidden:r}=e;const{navbar:{hideOnScroll:i},docs:{sidebar:{hideable:c}}}=(0,v.L)();return n.createElement("div",{className:(0,l.default)(G.sidebar,i&&G.sidebarWithHideableNavbar,r&&G.sidebarHidden)},i&&n.createElement(k.Z,{tabIndex:-1,className:G.sidebarLogo}),n.createElement(q,{path:t,sidebar:a}),c&&n.createElement(N,{onClick:o}))}const O=n.memo(Y);var X=a(13102),J=a(93163);const Q=e=>{let{sidebar:t,path:a}=e;const o=(0,J.e)();return n.createElement("ul",{className:(0,l.default)(r.k.docs.docSidebarMenu,"menu__list")},n.createElement(K,{items:t,activePath:a,onItemClick:e=>{"category"===e.type&&e.href&&o.toggle(),"link"===e.type&&o.toggle()},level:1}))};function $(e){return n.createElement(X.Zo,{component:Q,props:e})}const ee=n.memo($);function te(e){const t=(0,g.i)(),a="desktop"===t||"ssr"===t,l="mobile"===t;return n.createElement(n.Fragment,null,a&&n.createElement(O,e),l&&n.createElement(ee,e))}const ae={expandButton:"expandButton_m80_",expandButtonIcon:"expandButtonIcon_BlDH"};function ne(e){let{toggleSidebar:t}=e;return n.createElement("div",{className:ae.expandButton,title:(0,u.translate)({id:"theme.docs.sidebar.expandButtonTitle",message:"Expand sidebar",description:"The ARIA label and title attribute for expand button of doc sidebar"}),"aria-label":(0,u.translate)({id:"theme.docs.sidebar.expandButtonAriaLabel",message:"Expand sidebar",description:"The ARIA label and title attribute for expand button of doc sidebar"}),tabIndex:0,role:"button",onKeyDown:t,onClick:t},n.createElement(C,{className:ae.expandButtonIcon}))}const le={docSidebarContainer:"docSidebarContainer_b6E3",docSidebarContainerHidden:"docSidebarContainerHidden_b3ry",sidebarViewport:"sidebarViewport_Xe31"};function oe(e){let{children:t}=e;const a=(0,d.V)();return n.createElement(n.Fragment,{key:a?.name??"noSidebar"},t)}function re(e){let{sidebar:t,hiddenSidebarContainer:a,setHiddenSidebarContainer:o}=e;const{pathname:i}=(0,f.TH)(),[c,s]=(0,n.useState)(!1),d=(0,n.useCallback)((()=>{c&&s(!1),o((e=>!e))}),[o,c]);return n.createElement("aside",{className:(0,l.default)(r.k.docs.docSidebarContainer,le.docSidebarContainer,a&&le.docSidebarContainerHidden),onTransitionEnd:e=>{e.currentTarget.classList.contains(le.docSidebarContainer)&&a&&s(!0)}},n.createElement(oe,null,n.createElement("div",{className:(0,l.default)(le.sidebarViewport,c&&le.sidebarViewportHidden)},n.createElement(te,{sidebar:t,path:i,onCollapse:d,isHidden:c}),c&&n.createElement(ne,{toggleSidebar:d}))))}const ie={docMainContainer:"docMainContainer_gTbr",docMainContainerEnhanced:"docMainContainerEnhanced_Uz_u",docItemWrapperEnhanced:"docItemWrapperEnhanced_czyv"};function ce(e){let{hiddenSidebarContainer:t,children:a}=e;const o=(0,d.V)();return n.createElement("main",{className:(0,l.default)(ie.docMainContainer,(t||!o)&&ie.docMainContainerEnhanced)},n.createElement("div",{className:(0,l.default)("container padding-top--md padding-bottom--lg",ie.docItemWrapper,t&&ie.docItemWrapperEnhanced)},a))}const se={docPage:"docPage__5DB",docsWrapper:"docsWrapper_BCFX"};function de(e){let{children:t}=e;const a=(0,d.V)(),[l,o]=(0,n.useState)(!1);return n.createElement(m.Z,{wrapperClassName:se.docsWrapper},n.createElement(E,null),n.createElement("div",{className:se.docPage},a&&n.createElement(re,{sidebar:a.items,hiddenSidebarContainer:l,setHiddenSidebarContainer:o}),n.createElement(ce,{hiddenSidebarContainer:l},t)))}var me=a(4972),ue=a(90197);function be(e){const{versionMetadata:t}=e;return n.createElement(n.Fragment,null,n.createElement(ue.Z,{version:t.version,tag:(0,i.os)(t.pluginId,t.version)}),n.createElement(o.d,null,t.noIndex&&n.createElement("meta",{name:"robots",content:"noindex, nofollow"})))}function pe(e){const{versionMetadata:t}=e,a=(0,c.hI)(e);if(!a)return n.createElement(me.default,null);const{docElement:i,sidebarName:m,sidebarItems:u}=a;return n.createElement(n.Fragment,null,n.createElement(be,e),n.createElement(o.FG,{className:(0,l.default)(r.k.wrapper.docsPages,r.k.page.docsDocPage,e.versionMetadata.className)},n.createElement(s.q,{version:t},n.createElement(d.b,{name:m,items:u},n.createElement(de,null,i)))))}},4972:(e,t,a)=>{a.r(t),a.d(t,{default:()=>i});var n=a(67294),l=a(95999),o=a(1944),r=a(39483);function i(){return n.createElement(n.Fragment,null,n.createElement(o.d,{title:(0,l.translate)({id:"theme.NotFound.title",message:"Page Not Found"})}),n.createElement(r.Z,null,n.createElement("main",{className:"container margin-vert--xl"},n.createElement("div",{className:"row"},n.createElement("div",{className:"col col--6 col--offset-3"},n.createElement("h1",{className:"hero__title"},n.createElement(l.default,{id:"theme.NotFound.title",description:"The title of the 404 page"},"Page Not Found")),n.createElement("p",null,n.createElement(l.default,{id:"theme.NotFound.p1",description:"The first paragraph of the 404 page"},"We could not find what you were looking for.")),n.createElement("p",null,n.createElement(l.default,{id:"theme.NotFound.p2",description:"The 2nd paragraph of the 404 page"},"Please contact the owner of the site that linked you to the original URL and let them know their link is broken.")))))))}},74477:(e,t,a)=>{a.d(t,{E:()=>i,q:()=>r});var n=a(67294),l=a(902);const o=n.createContext(null);function r(e){let{children:t,version:a}=e;return n.createElement(o.Provider,{value:a},t)}function i(){const e=(0,n.useContext)(o);if(null===e)throw new l.i6("DocsVersionProvider");return e}}}]);