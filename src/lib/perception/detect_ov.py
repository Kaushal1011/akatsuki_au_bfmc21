from typing import List, Optional, Tuple
import cv2
import numpy as np
from openvino.runtime import Core
import time

FONT = "Arial.ttf"  # https://ultralytics.com/assets/Arial.ttf

AREA_THRESHOLD = {
    'car':5000,
    'crosswalk':6200,
    'doll':5000,
    'highway_entry':5300,
    'highway_exit':5000,
    'no_entry':6000,
    'onewayroad':7000,
    'parking':10000,
    'pedestrian':50,
    'priority':11000,
    'roadblock':50,
    'roundabout':6000,
    'stop':5500,
    'trafficlight':50
}


class Colors:
    # Ultralytics color palette https://ultralytics.com/
    def __init__(self):
        # hex = matplotlib.colors.TABLEAU_COLORS.values()
        hex = (
            "FF3838",
            "FF9D97",
            "FF701F",
            "FFB21D",
            "CFD231",
            "48F90A",
            "92CC17",
            "3DDB86",
            "1A9334",
            "00D4BB",
            "2C99A8",
            "00C2FF",
            "344593",
            "6473FF",
            "0018EC",
            "8438FF",
            "520085",
            "CB38FF",
            "FF95C8",
            "FF37C7",
        )
        self.palette = [self.hex2rgb("#" + c) for c in hex]
        self.n = len(self.palette)

    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c

    @staticmethod
    def hex2rgb(h):  # rgb order (PIL)
        return tuple(int(h[1 + i : 1 + i + 2], 16) for i in (0, 2, 4))


def is_ascii(s=""):
    # Is string composed of all ASCII (no UTF) characters? (note str().isascii() introduced in python 3.7)
    s = str(s)  # convert list, tuple, None, etc. to str
    return len(s.encode().decode("ascii", "ignore")) == len(s)


colors = Colors()  # create instance for 'from utils.plots import colors'


class Annotator:
    # YOLOv5 Annotator for train/val mosaics and jpgs and detect/hub inference annotations
    def __init__(
        self,
        im,
        line_width=None,
        font_size=None,
        font="Arial.ttf",
        pil=False,
        example="abc",
    ):
        assert (
            im.data.contiguous
        ), "Image not contiguous. Apply np.ascontiguousarray(im) to Annotator() input images."
        non_ascii = not is_ascii(
            example
        )  # non-latin labels, i.e. asian, arabic, cyrillic
        self.pil = pil or non_ascii
        self.im = im
        self.lw = line_width or max(round(sum(im.shape) / 2 * 0.003), 2)  # line width

    def box_label(
        self, box, label="", color=(128, 128, 128), txt_color=(255, 255, 255)
    ):
        # Add one xyxy box to image with label
        if self.pil or not is_ascii(label):
            self.draw.rectangle(box, width=self.lw, outline=color)  # box
            if label:
                w, h = self.font.getsize(label)  # text width, height
                outside = box[1] - h >= 0  # label fits outside box
                self.draw.rectangle(
                    (
                        box[0],
                        box[1] - h if outside else box[1],
                        box[0] + w + 1,
                        box[1] + 1 if outside else box[1] + h + 1,
                    ),
                    fill=color,
                )
                # self.draw.text((box[0], box[1]), label, fill=txt_color, font=self.font, anchor='ls')  # for PIL>8.0
                self.draw.text(
                    (box[0], box[1] - h if outside else box[1]),
                    label,
                    fill=txt_color,
                    font=self.font,
                )
        else:  # cv2
            p1, p2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
            cv2.rectangle(
                self.im, p1, p2, color, thickness=self.lw, lineType=cv2.LINE_AA
            )
            if label:
                tf = max(self.lw - 1, 1)  # font thickness
                w, h = cv2.getTextSize(label, 0, fontScale=self.lw / 3, thickness=tf)[
                    0
                ]  # text width, height
                outside = p1[1] - h - 3 >= 0  # label fits outside box
                p2 = p1[0] + w, p1[1] - h - 3 if outside else p1[1] + h + 3
                cv2.rectangle(self.im, p1, p2, color, -1, cv2.LINE_AA)  # filled
                cv2.putText(
                    self.im,
                    label,
                    (p1[0], p1[1] - 2 if outside else p1[1] + h + 2),
                    0,
                    self.lw / 3,
                    txt_color,
                    thickness=tf,
                    lineType=cv2.LINE_AA,
                )

    def rectangle(self, xy, fill=None, outline=None, width=1):
        # Add rectangle to image (PIL-only)
        self.draw.rectangle(xy, fill, outline, width)

    def text(self, xy, text, txt_color=(255, 255, 255)):
        # Add text to image (PIL-only)
        w, h = self.font.getsize(text)  # text width, height
        self.draw.text((xy[0], xy[1] - h + 1), text, fill=txt_color, font=self.font)

    def result(self):
        # Return annotated image as array
        return np.asarray(self.im)


def xywh2xyxy(x):
    # Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def nms(dets, scores, thresh):
    """
    dets is a numpy array : num_dets, 4
    scores ia  nump array : num_dets,
    """
    x1 = dets[:, 0]
    y1 = dets[:, 1]
    x2 = dets[:, 2]
    y2 = dets[:, 3]

    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores.argsort()[::-1]  # get boxes with more ious first

    keep = []
    while order.size > 0:
        i = order[0]  # pick maxmum iou box
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1 + 1)  # maximum width
        h = np.maximum(0.0, yy2 - yy1 + 1)  # maxiumum height
        inter = w * h
        ovr = inter / (areas[i] + areas[order[1:]] - inter)

        inds = np.where(ovr <= thresh)[0]
        order = order[inds + 1]

    return np.array(keep)


def scale_coords(img1_shape, coords, img0_shape, ratio_pad=None):
    # Rescale coords (xyxy) from img1_shape to img0_shape
    if ratio_pad is None:  # calculate from img0_shape
        gain = min(
            img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1]
        )  # gain  = old / new
        pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (
            img1_shape[0] - img0_shape[0] * gain
        ) / 2  # wh padding
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    coords[:, [0, 2]] -= pad[0]  # x padding
    coords[:, [1, 3]] -= pad[1]  # y padding
    coords[:, :4] /= gain
    coords = clip_coords(coords, img0_shape)
    return coords


def clip_coords(boxes, shape):
    # Clip bounding xyxy bounding boxes to image shape (height, width)
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, shape[1])  # x1, x2
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, shape[0])  # y1, y2
    return boxes


def non_max_suppression_np(
    prediction,
    conf_thres=0.25,
    iou_thres=0.45,
    classes=None,
    agnostic=False,
    multi_label=False,
    labels=(),
    max_det=300,
):
    """Non-Maximum Suppression (NMS) on inference results to reject overlapping bounding boxes

    Returns:
         list of detections, on (n,6) tensor per image [xyxy, conf, cls]
    """

    bs = prediction.shape[0]  # batch size
    nc = prediction.shape[2] - 5  # number of classes
    xc = prediction[..., 4] > conf_thres  # candidates

    # Checks
    assert (
        0 <= conf_thres <= 1
    ), f"Invalid Confidence threshold {conf_thres}, valid values are between 0.0 and 1.0"
    assert (
        0 <= iou_thres <= 1
    ), f"Invalid IoU {iou_thres}, valid values are between 0.0 and 1.0"

    # Settings
    # min_wh = 2  # (pixels) minimum box width and height
    max_wh = 7680  # (pixels) maximum box width and height
    max_nms = 30000  # maximum number of boxes into torchvision.ops.nms()
    time_limit = 0.1 + 0.03 * bs  # seconds to quit after
    redundant = True  # require redundant detections
    multi_label &= nc > 1  # multiple labels per box (adds 0.5ms/img)
    merge = False  # use merge-NMS

    t = time.time()
    output = [np.zeros((0, 6))] * bs
    for xi, x in enumerate(prediction):  # image index, image inference
        # Apply constraints
        # x[((x[..., 2:4] < min_wh) | (x[..., 2:4] > max_wh)).any(1), 4] = 0  # width-height
        x = x[xc[xi]]  # confidence

        # Cat apriori labels if autolabelling
        if labels and len(labels[xi]):
            lb = labels[xi]
            v = np.zeros((len(lb), nc + 5))
            v[:, :4] = lb[:, 1:5]  # box
            v[:, 4] = 1.0  # conf
            v[range(len(lb)), lb[:, 0].long() + 5] = 1.0  # cls
            x = np.concatenate((x, v), 0)

        # If none remain process next image
        if not x.shape[0]:
            continue

        # Compute conf
        x[:, 5:] *= x[:, 4:5]  # conf = obj_conf * cls_conf

        # Box (center x, center y, width, height) to (x1, y1, x2, y2)
        box = xywh2xyxy(x[:, :4])
        # Detections matrix nx6 (xyxy, conf, cls)
        if multi_label:
            i, j = (x[:, 5:] > conf_thres).nonzero(as_tuple=False).T
            x = np.concatenate(
                (box[i], x[i, j + 5, None], j[:, None].astype("float16")), 1
            )
        else:  # best class only
            k: np.ndarray = x[:, 5:]

            conf, j = k.max(1, keepdims=True), k.argmax(1).reshape(k.shape[0], 1)

            x = np.concatenate((box, conf, j.astype("float16")), 1)[
                conf.ravel() > conf_thres
            ]

        # Filter by class
        if classes is not None:
            x = x[(x[:, 5:6] == np.array(classes)).any(1)]

        # Apply finite constraint
        # if not torch.isfinite(x).all():
        #     x = x[torch.isfinite(x).all(1)]

        # Check shape
        n = x.shape[0]  # number of boxes
        if not n:  # no boxes
            continue
        elif n > max_nms:  # excess boxes
            x = x[x[:, 4].argsort(descending=True)[:max_nms]]  # sort by confidence

        # Batched NMS
        c = x[:, 5:6] * (0 if agnostic else max_wh)  # classes
        boxes, scores = x[:, :4] + c, x[:, 4]  # boxes (offset by class), scores
        scores = scores.astype("float16")
        # print("boxes", boxes,"scores", scores,"iou_thres", iou_thres)
        i = nms(boxes, scores, iou_thres)  # NMS
        if i.shape[0] > max_det:  # limit detections
            i = i[:max_det]

        output[xi] = x[i]
        if (time.time() - t) > time_limit:
            print(f"WARNING: NMS time limit {time_limit:.3f}s exceeded")
            break  # time limit exceeded

    return output


def get_area(bbox: np.ndarray) -> list:
    # (x1, y1, x2, y2)
    return ((bbox[:, 2] - bbox[:, 0]) * (bbox[:, 3] - bbox[:, 1])).tolist()


def roi_func(img: np.ndarray) -> np.ndarray:
    """Given image get Region of interest

    Args:
        img: (np.ndarray) input image

    Returns:
        np.ndarray: the roi image
    """
    luroi = 0.15
    ruroi = 1
    lbroi = 0.15
    rbroi = 1
    hroi = 0.05
    broi = 0
    # create stencil just the first time and then save for later use
    roi = [
            (
                int(luroi * ((img.shape[1] - 1))),
                int(hroi * (img.shape[0] - 1)),
            ),
            (
                int(lbroi * ((img.shape[1] - 1))),
                int((img.shape[0] - 1) * (1 - broi)),
            ),
            (
                int(rbroi * ((img.shape[1] - 1))),
                int((img.shape[0] - 1) * (1 - broi)),
            ),
            (
                int(ruroi * ((img.shape[1] - 1))),
                int(hroi * (img.shape[0] - 1)),
            ),
        ]
    stencil = np.zeros_like(img, dtype="uint8")
        # specify coordinates of the polygon
    polygon = np.array(roi)

        # fill polygon with ones
    cv2.fillConvexPoly(stencil, polygon, [255, 255, 255])

    img = cv2.bitwise_and(img, img, mask=stencil[:, :, 0])
    return img


map2label = ['car', 'crosswalk', 'doll', 'highway_entry', 'highway_exit', 'no_entry', 'onewayroad', 'parking', 'pedestrian', 'priority', 'roadblock', 'roundabout', 'stop', 'trafficlight']


class Detection:
    def __init__(
        self, model_path: str = "best1yet_openvino_model/best1yet.xml"
    ) -> None:
        ie = Core()
        model = ie.read_model(model=model_path)
        device = "MYRIAD" if "MYRIAD" in ie.available_devices else "CPU"
        self.compiled_model = ie.compile_model(model=model, device_name=device)
        print(f">>> Loaded model in {device}.")
        self.input_layer_ir = next(iter(self.compiled_model.inputs))
        N, C, H, W = self.input_layer_ir.shape
        assert H == 640 and W == 640, ""

    def __call__(
        self, img: np.ndarray, bbox: bool = False
    ) -> Tuple[List[float], List[float], Optional[np.ndarray]]:
        img = roi_func(img)
        if not img.shape == (640, 640, 3):
            img_resized = cv2.resize(img, (640, 640))

        x: np.ndarray = img_resized / 255
        x = x.astype(np.float32)
        x = np.expand_dims(x.transpose(2, 0, 1), 0)
        request = self.compiled_model.create_infer_request()
        request.infer({self.input_layer_ir.any_name: x})
        pred = request.get_tensor("output").data
        pred_nms = non_max_suppression_np(pred)
        if len(pred_nms) == 0:
            if bbox:
                return [], [], None
            return [], []

        pred_nms = pred_nms[0]
        pred_nms[:, :4] = scale_coords(x.shape[2:], pred_nms[:, :4], img.shape).round()
        classes = np.unique(pred_nms[:, -1]).tolist()
        area = get_area(pred_nms[:, :4])
        if bbox:
            out_image = self.draw_bbox(pred_nms, classes=classes, image=img)
            classes = [map2label[int(x)] for x in classes]
            detections = [
                (c, a) for c, a in zip(classes, area) if a > AREA_THRESHOLD[c]
            ]
            detections.append()
            return (detections, out_image)
        else:
            if classes:
                classes = [map2label[int(x)] for x in classes]
                return [(c, a) for c, a in zip(classes, area) if a > AREA_THRESHOLD[c]]
            return []

    def draw_bbox(
        self, pred_nms: np.ndarray, classes: Tuple[int], image: np.ndarray
    ) -> np.ndarray:
        """Draw bbox on image"""
        annotator = Annotator(image, line_width=2)
        for c in classes:
            n = (pred_nms[:, -1] == c).sum()  # detections per class
            # s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

        for *xyxy, conf, cls in reversed(pred_nms):
            c = int(cls)
            label = f"{map2label[c]} {conf:.2f}"
            annotator.box_label(xyxy, label, colors(c, True))

        im0 = annotator.result()
        # TODO: change channels if required
        im0 = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # cv2.imwrite("output101.jpg", im0)
        return im0
