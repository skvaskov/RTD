%%saveimage(string,[width,length])
function saveimage(a,varargin)
   %%%%Mode 1: changes axes
   %%%%Mode 2: Old Method
   %%%%Mode 3: Same as Mode 1
    width_pic=6;
    length_pic=4;
    mode=1;
    border=0;
    topborder=0;
    form='pdf';
    if ~isempty(varargin)>0
        for i=1:2:length(varargin)
            switch lower(varargin{i})
                case 'size'
                    size=varargin{i+1};
                    width_pic=size(1);
                    length_pic=size(2);
                case 'mode' 
                    mode=varargin{i+1};
                case 'border'
                    border=varargin{i+1};
                case 'format'
                    form = varargin{i+1};
                case 'topborder'
                    topborder=varargin{i+1};
            end
        end
    end
   
    if mode==(1 || 3)    
        set(gcf,'Units','Inches')
        set(gcf,'Position',[4 4 width_pic-border/2 length_pic-border/2])
        set(gca,'units','inches')

        pause(.05)

%         ti = get(gca,'TightInset');
        pause(.05)
        set(gca, 'XTickMode', 'manual');
        set(gca, 'YTickMode', 'manual');
        pause(.05);
%         set(gca,'Position',[ti(1)+border/2 ti(2)+border/2 width_pic-ti(3)-ti(1)-border/2 length_pic-ti(4)-ti(2)-border/2]);
%         set(gca,'Position',[ti(1) ti(2) width_pic-ti(3)-ti(1) length_pic-ti(4)-ti(2)]);
        pause(.05)
        pos = get(gca,'Position');
        ti = get(gca,'TightInset');
        set(gcf, 'PaperUnits','inches');
        %set(gcf, 'PaperSize', [pos(3)+ti(1)+ti(3) pos(4)+ti(2)+ti(4)]);
        set(gcf, 'PaperSize', [width_pic length_pic]);
        set(gcf, 'PaperPositionMode', 'manual');
        set(gcf, 'PaperPosition', [0 0 width_pic length_pic]);
        %set(gcf, 'PaperPosition',[0 0 pos(3)+ti(1)+ti(3) pos(4)+ti(2)+ti(4)]);
        print(strcat('-d',form),a);
    else    
        set(gcf, 'PaperPositionMode', 'manual');
        set(gcf, 'PaperUnits', 'inches');
        set(gcf, 'PaperPosition', [0 0 width_pic length_pic])
        set(gcf, 'PaperSize', [width_pic length_pic]);
        
        set(gca, 'FontSize', 15);
        set(gca, 'Layer', 'top')
        
        saveas(gcf,strcat(a),form)
    end
