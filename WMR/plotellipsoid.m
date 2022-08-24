function varargout=plotellipsoid(varargin)
%This function plots an ellipsoid that encases the human demonstrations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2019 Iman Salehi, Robotics and Controls Lab        %%%
%%%                     ECE, UConn, Connecticut, USA                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    error(nargchk(4, 6, nargin, 'struct'));
    error(nargchk(0, 1, nargout, 'struct'));
    
    % Parse and check inputs
    if ishandle(varargin{1})
        hAx = varargin{1};
        varargin(1) = [];
    else
        hAx = gca();
    end
    

    % Ellipse centre
    center = varargin{1};
    center = center(:);
    if length(center) ~= 3
        error('plotellipsoid:InvalidCentre', ...
            'Ellipsoid center must be a 3 element column vector');
    end

    a = varargin{2};
    b = varargin{3};
    c = varargin{4};
    
    varargin(1:4) = [];

    % See if a linespec is supplied
    if ~isempty(varargin)
        linespec1 = varargin{1,1};
        linespec2 = varargin{1,2};
        
    else
        linespec = '';
    end


    % form the parameter vector
    npts = 100;
    theta = (-npts:2:npts)/npts*pi;
    phi = (-npts:2:npts)'/npts*pi/2;

    cosphi = cos(phi); cosphi(1) = 0; cosphi(npts+1) = 0;
    sintheta = sin(theta); sintheta(1) = 0; sintheta(npts+1) = 0;

    x = cosphi*cos(theta);
    y = cosphi*sintheta;
    z = sin(phi)*ones(1,npts+1);
    
    % Ellipsoid points

    newX = a*x+center(1);
    newY = b*y+center(2);
    newZ = c*z+center(3);
    
    % The actual plotting 
    h = plot3(hAx,newX,newY,newZ,'Color',linespec1,'LineWidth',linespec2);

    % Return the handle if asked for
    if nargout == 1
        varargout = {h};
    end
    
end